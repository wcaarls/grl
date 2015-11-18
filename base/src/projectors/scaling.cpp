/** \file scaling.cpp
 * \brief Scaling projector source file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-09-02
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
#include <grl/projectors/scaling.h>

using namespace grl;

REGISTER_CONFIGURABLE(ScalingProjector)

void ScalingProjector::request(const std::string &role, ConfigurationRequest *config)
{
  config->push_back(CRP("scaling", "Scaling vector", scaling_));
  config->push_back(CRP("projector", "projector." + role, "Downstream projector", projector_));
}

void ScalingProjector::configure(Configuration &config)
{
  projector_ = (Projector*) config["projector"].ptr();
  scaling_ = config["scaling"];
}

void ScalingProjector::reconfigure(const Configuration &config)
{
}

ScalingProjector *ScalingProjector::clone() const
{
  ScalingProjector *projector = new ScalingProjector(*this);
  projector->projector_ = projector_->clone();
  return projector;
}

ProjectionPtr ScalingProjector::project(const Vector &in) const
{
  if (in.size() != scaling_.size())
    throw bad_param("projector/pre/scaling:scaling");
    
  return projector_->project(in*scaling_);
}

Matrix ScalingProjector::jacobian(const Vector &in) const
{
  if (in.size() != scaling_.size())
    throw bad_param("projector/pre/scaling:scaling");

  return projector_->jacobian(in*scaling_)*diagonal(scaling_);
}
