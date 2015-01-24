/** \file tile_coding.cpp
 * \brief Tile coding projector source file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-01-22
 *
 * \copyright \verbatim
 * Copyright (c) 2015, Wouter Caarls
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

#include <grl/projectors/tile_coding.h>

using namespace grl;

REGISTER_CONFIGURABLE(TileCodingProjector)

void TileCodingProjector::request(ConfigurationRequest *config)
{
  config->push_back(CRP("tilings", "Number of tilings", tilings_));
  config->push_back(CRP("memory", "Hash table size", memory_));
  
  config->push_back(CRP("resolution", "Size of a single tile", resolution_));
  config->push_back(CRP("wrapping", "Wrapping boundaries (must be multiple of resolution)", wrapping_));
}

void TileCodingProjector::configure(Configuration &config)
{
  tilings_ = config["tilings"];
  memory_ = config["memory"];
  
  resolution_ = config["resolution"];
  wrapping_ = config["wrapping"];

  if (wrapping_.empty())
    wrapping_.resize(resolution_.size(), 0.);
    
  if (wrapping_.size() != resolution_.size())
    throw bad_param("projector/tile_coding:wrapping");
  
  for (size_t ii=0; ii < resolution_.size(); ++ii)
  {
    scaling_.push_back(tilings_/resolution_[ii]);
    wrapping_[ii] *= scaling_[ii];
    if (fabs(wrapping_[ii]-round(wrapping_[ii])) > 0.001)
      throw bad_param("projector/tile_coding:wrapping");
    wrapping_[ii] = round(wrapping_[ii]);
  }
}

void TileCodingProjector::reconfigure(const Configuration &config)
{
}

TileCodingProjector *TileCodingProjector::clone() const
{
  TileCodingProjector *tc = new TileCodingProjector();
  tc->tilings_ = tilings_;
  tc->memory_ = memory_;
  tc->scaling_ = scaling_;
  tc->wrapping_ = wrapping_;
  
  return tc;
}

#define MAX_NUM_VARS 32

ProjectionPtr TileCodingProjector::project(const Vector &in) const
{
  int num_floats = in.size();
  int i,j;
  int qstate[MAX_NUM_VARS];
  int base[MAX_NUM_VARS];
  int coordinates[MAX_NUM_VARS + 1];   /* one interval number per relevant dimension */
  int num_coordinates = num_floats + 1;
  
  if (num_floats != (int)scaling_.size())
    throw bad_param("representation/tile_coding:resolution");
  
  IndexProjection *p = new IndexProjection();
  p->indices.resize(tilings_);

  /* quantize state to integers (henceforth, tile widths == numTilings) */
  for (i = 0; i < num_floats; i++)
  {
    qstate[i] = (int) floor(in[i] * scaling_[i]);
    base[i] = 0;
  }

  /*compute the tile numbers */
  for (j = 0; j < tilings_; j++)
  {
    /* loop over each relevant dimension */
    for (i = 0; i < num_floats; i++)
    {
      /* find coordinates of activated tile in tiling space */
      coordinates[i] = qstate[i] - safe_mod((qstate[i] - base[i]), tilings_);

      if (wrapping_[i] != 0)
        coordinates[i] = safe_mod(coordinates[i], (int)wrapping_[i]);

      /* compute displacement of next tiling in quantized space */
      base[i] += 1 + (2 * i);
    }
    /* add additional indices for tiling and hashing_set so they hash differently */
    coordinates[i] = j;
    p->indices[j] = getFeatureLocation(coordinates, num_coordinates);
  }
  
  return ProjectionPtr(p);
}
