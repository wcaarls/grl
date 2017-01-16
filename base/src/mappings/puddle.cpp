/** \file puddle.cpp
 * \brief Puddle mapping source file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2017-01-10
 *
 * \copyright \verbatim
 * Copyright (c) 2017, Wouter Caarls
 * All rights reserved.
 *
 * This file is part of GRL, the Generic Reinforcement Learning library.
 *
 * GRL is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * \endverbatim
 */

#include <grl/mappings/puddle.h>

using namespace grl;

REGISTER_CONFIGURABLE(PuddleMapping)

void PuddleMapping::request(ConfigurationRequest *config)
{
  config->push_back(CRP("seed", "World seed", (int)seed_));
  config->push_back(CRP("smoothing", "Standard deviation of Gaussian filter", smoothing_, CRP::Configuration, 0., 10.));
  config->push_back(CRP("steepness", "Parameter of sigmoid stretching", steepness_, CRP::Configuration, 1., 50.));
}

void PuddleMapping::configure(Configuration &config)
{
  seed_ = config["seed"];
  smoothing_ = config["smoothing"];
  steepness_ = config["steepness"];
  
  INFO("Generating world from seed " << seed_);

  Rand rand;
  if (seed_ != 0)
    rand.init(seed_);

  // Calculate filter size  
  size_t fsz = map_.rows()*(2*2.96*smoothing_); // Get 99% of univariate distribution
  fsz = fsz + fsz%2;                            // Make odd
    
  // Randomize map
  Matrix m(map_.rows()+fsz-1, map_.cols()+fsz-1);
  for (size_t ii=0; ii < m.rows(); ++ii)
    for (size_t jj=0; jj < m.cols(); ++jj)
      m(ii, jj) = rand.getNormal(0, 1);
  
  if (fsz)
  {
    // Create filter
    Eigen::ArrayXXd f(fsz, fsz);
    for (size_t ii=0; ii < fsz; ++ii)
      for (size_t jj=0; jj < fsz; ++jj)
      {
        double x = (ii-(fsz-1.)/2)/map_.rows(), y = (jj-(fsz-1.)/2)/map_.cols();
      
        f(ii, jj) = exp(-(x*x+y*y)/(2*smoothing_*smoothing_));
      }
      
    // Filter
    for (size_t ii=0; ii < map_.rows(); ++ii)
      for (size_t jj=0; jj < map_.cols(); ++jj)
        map_(ii, jj) = (m.block(ii, jj, fsz, fsz).array()*f).mean();
  }
      
  // Normalize
  double minv = map_.minCoeff(), maxv = map_.maxCoeff();
  map_ = 2*((map_.array()-minv) / (maxv-minv))-1;

  // Stretch
  map_ = (1.+(-steepness_*map_.array()).exp()).inverse();
      
  // Renormalize
  minv = map_.minCoeff(), maxv = map_.maxCoeff();
  map_ = (map_.array()-minv) / (maxv-minv);
}

void PuddleMapping::reconfigure(const Configuration &config)
{
}

double PuddleMapping::read(const Vector &in, Vector *result) const
{
  if (in.size() != 2)
    throw Exception("mapping/puddle only works in two dimensions");

  result->resize(1);
  
  // Convert unit into map coordinates. Note that the map defines depths for
  // grid intersections, so 1 is mapped to the last row/column index.
  double lx = in[0]*(map_.cols()-1),
         ly = in[1]*(map_.rows()-1);
         
  // Calculate which grid cell the coordinate falls into. This is simply the
  // map coordinate rounded down. Make sure the last row/column index can never
  // be reached.
  size_t mx = fmin(fmax(lx, 0.), map_.cols()-2.),
         my = fmin(fmax(ly, 0.), map_.rows()-2.);
         
  // Calculate offset within our grid cell.
  double dx = fmin(fmax(lx - mx, 0.), 1.),
         dy = fmin(fmax(ly - my, 0.), 1.);
         
  // Bilinear interpolation counting from lower left corner at (mx, my).
  double depth = map_(my,mx)    *(1-dx)*(1-dy) +
                 map_(my,mx+1)  *   dx *(1-dy) +
                 map_(my+1,mx)  *(1-dx)*   dy  +
                 map_(my+1,mx+1)*   dx *   dy;
                 
  return (*result)[0] = depth;
}
