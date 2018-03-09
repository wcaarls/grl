/** \file timeline.cpp
 * \brief Imported timeline mapping source file. 
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2018-03-08
 *
 * \copyright \verbatim
 * Copyright (c) 2018, Wouter Caarls
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

#include <grl/mappings/image.h>
#include <libics.h>
#include <libics_test.h>

using namespace grl;

REGISTER_CONFIGURABLE(ImageMapping)

void ImageMapping::request(ConfigurationRequest *config)
{
  config->push_back(CRP("file", "ICS image file name", file_));
  config->push_back(CRP("min", "Lower domain limit", min_));
  config->push_back(CRP("max", "Upper domain limit", max_));
}

void ImageMapping::configure(Configuration &config)
{
  file_ = config["file"].str();
  min_ = config["min"].v();
  max_ = config["max"].v();
  
  ICS *ics;
  Ics_Error err;
  
  if ((err = IcsOpen(&ics, file_.c_str(), "r")))
  {
    ERROR("Cannot open ICS image '" << file_ << "': " << IcsGetErrorText(err));
    throw bad_param("mapping/image:file");
  }
  
  if (grl_log_verbosity__ >= 5)
  {
    CRAWL("Image information:");
    IcsPrintIcs(ics);
  }
    
  Ics_DataType dt;
  int ndims;
  size_t dims[GRL_STATIC_VECTOR_SIZE];
  
  IcsGetLayout(ics, &dt, &ndims, dims);
  if (dt != Ics_real64)
  {
    ERROR("ICS image '" << file_ << "' has wrong data type " << dt << ". Must be double.");
    IcsClose(ics);
    throw bad_param("mapping/image:file");
  }
  
  dims_.resize(ndims);
  for (size_t ii=0; ii < ndims; ++ii)
    dims_[ii] = dims[ii];
  
  data_ = (double*) malloc(IcsGetDataSize(ics));
  if ((err = IcsGetData(ics, data_, IcsGetDataSize(ics))))
  {
    ERROR("Cannot read ICS image '" << file_ << "': " << IcsGetErrorText(err));
    IcsClose(ics);
    throw bad_param("mapping/image:file");
  }
  
  if (min_.size() != max_.size())
  {
    IcsClose(ics);
    throw bad_param("mapping/image:{min,max}");
  }
  
  // TODO: relax
  if (min_.size() != 2)
  {
    ERROR("Only two-dimensional mappings are supported");
    throw bad_param("mapping/image:{min,max}");
  }
}

void ImageMapping::reconfigure(const Configuration &config)
{
}

double ImageMapping::read(const Vector &in, Vector *result) const
{
  if (in.size() != min_.size())
    throw Exception("Input dimensionality does not match image");

  result->resize(1);
  
  // Convert input into map coordinates. Note that the map defines depths for
  // grid intersections, so the max is mapped to the last row/column index.
  double lx = (in[0]-min_[0])/(max_[0]-min_[0])*(dims_[0]-1),
         ly = (in[1]-min_[1])/(max_[1]-min_[1])*(dims_[1]-1);
         
  // Calculate which grid cell the coordinate falls into. This is simply the
  // map coordinate rounded down. Make sure the last row/column index can never
  // be reached.
  size_t mx = fmin(fmax(lx, 0.), dims_[0]-2.),
         my = fmin(fmax(ly, 0.), dims_[1]-2.);
         
  // Calculate offset within our grid cell.
  double dx = fmin(fmax(lx - mx, 0.), 1.),
         dy = fmin(fmax(ly - my, 0.), 1.);
         
  // Bilinear interpolation counting from lower left corner at (mx, my).
  double depth = data_[my*dims_[0]+mx]       *(1-dx)*(1-dy) +
                 data_[my*dims_[0]+mx+1]     *   dx *(1-dy) +
                 data_[(my+1)*dims_[0]+mx]   *(1-dx)*   dy  +
                 data_[(my+1)*dims_[0]+mx+1] *   dx *   dy;
                 
  return (*result)[0] = depth;
}
