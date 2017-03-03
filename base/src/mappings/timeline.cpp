/** \file timeline.cpp
 * \brief Imported timeline mapping source file. 
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2017-01-12
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

#include <grl/mappings/timeline.h>

using namespace grl;

REGISTER_CONFIGURABLE(TimelineMapping)

void TimelineMapping::request(ConfigurationRequest *config)
{
  config->push_back(CRP("importer", "importer.dynamic", "Importer with time as the first column", importer_));
}

void TimelineMapping::configure(Configuration &config)
{
  importer_ = (Importer*)config["importer"].ptr();
  importer_->open();
  
  Vector line;
  while (importer_->read(&line))
  {
    if (data_.size() && data_[data_.size()-1].size() != line.size())
    {
      ERROR("Data vectors should be of equal size");
      throw bad_param("mapping/timeline:importer");
    }
    data_.push_back(line);
  }
    
  if (data_.empty())
  {
    ERROR("Could not import timeline");
    throw bad_param("mapping/timeline:importer");
  }
    
  if (data_[0][0] != 0.)
  {
    ERROR("Timeline does not start at 0");
    throw bad_param("mapping/timeline:importer");
  }
}

void TimelineMapping::reconfigure(const Configuration &config)
{
}

double TimelineMapping::read(const Vector &in, Vector *result) const
{
  if (in.size() != 1)
    throw Exception("mapping/timeline has only one input dimension");
    
  result->resize(data_[0].size()-1);
    
  double time = in[0];
  size_t &idx = *prev_idx_.instance();
  
  // Sanity check
  if (idx > data_.size())
    idx = data_.size();
    
  // Reset counting index if necessary
  if (idx > 0 && data_[idx-1][0] > time)
    idx = 0;
  
  // Find first index that is larger than current time  
  for (; idx < data_.size() && data_[idx][0] <= time; ++idx);

  Vector v;
  if (idx < data_.size())
  {
    // Interpolate
    double ti = (data_[idx][0]-time)/(data_[idx][0]-data_[idx-1][0]);
    v         = ti*data_[idx-1] + (1-ti)*data_[idx];
  }
  else
  {
    // Repeat last value forever
    v = data_.back();
  }

  // Remove time  
  for (size_t ii=0; ii < result->size(); ++ii)
    (*result)[ii] = v[ii+1];
    
  return (*result)[0];
}
