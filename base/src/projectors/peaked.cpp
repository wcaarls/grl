/** \file peaked.cpp
 * \brief Peaked projector source file.
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

#include <string.h>
#include <grl/projectors/peaked.h>

using namespace grl;

REGISTER_CONFIGURABLE(PeakedProjector)

void PeakedProjector::request(const std::string &role, ConfigurationRequest *config)
{
  config->push_back(CRP("peaking", "Extra resolution factor around center (offset by 1/factor at edges)", peaking_, CRP::Configuration));
  
  NormalizingProjector::request(role, config);
}

void PeakedProjector::configure(Configuration &config)
{
  NormalizingProjector::configure(config);
  
  peaking_ = config["peaking"];
  
  if (peaking_.size() != min_.size())
    throw bad_param("projector/peaked:peaking");

  scaling_ = 1./(max_-min_);
  range2_ = (max_-min_)/2.;
}

void PeakedProjector::reconfigure(const Configuration &config)
{
}

PeakedProjector *PeakedProjector::clone() const
{
  PeakedProjector *projector = new PeakedProjector(*this);
  projector->projector_ = projector_->clone();
  return projector;
}

ProjectionPtr PeakedProjector::project(const Vector &in) const
{
  // Scale input to [-1, 1], apply squashing, and rescale to range
  return projector_->project((squash(2.*(in-min_)*scaling_-1., peaking_)+1.)*range2_+min_);
}
