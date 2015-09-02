/** \file peaked.cpp
 * \brief Peaked discretizer source file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-09-01
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

#include <grl/discretizers/peaked.h>

using namespace grl;

REGISTER_CONFIGURABLE(PeakedDiscretizer)

void PeakedDiscretizer::request(const std::string &role, ConfigurationRequest *config)
{
  UniformDiscretizer::request(role, config);

  config->push_back(CRP("peaking", "Extra resolution factor around center (offset by 1/factor at edges)", peaking_, CRP::Configuration));
}

void PeakedDiscretizer::configure(Configuration &config)
{
  UniformDiscretizer::configure(config);

  peaking_ = config["peaking"];
  
  if (peaking_.size() != min_.size())
    throw bad_param("discretizer/peaked:peaking");

  for (size_t dd=0; dd < steps_.size(); ++dd)
  {
    // Create smooth [-1, 1] ramp
    for (size_t vv=0; vv < steps_[dd]; ++vv)
      values_[dd][vv] = 2.*vv/(steps_[dd]-1)-1;
      
    // Squash ramp and scale to range
    values_[dd] = (squash(values_[dd], -peaking_[dd])+1.)*(max_[dd]-min_[dd])/2+min_[dd];
  }
}

PeakedDiscretizer* PeakedDiscretizer::clone()
{
  return new PeakedDiscretizer(*this);
}
