/** \file normalizing.cpp
 * \brief Normalizing projector source file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-03-10
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
#include <grl/projectors/normalizing.h>

using namespace grl;

REGISTER_CONFIGURABLE(NormalizingProjector)

void NormalizingProjector::request(const std::string &role, ConfigurationRequest *config)
{
  if (role == "observation")
  {
    config->push_back(CRP("input_min", "vector.observation_min", "Lower input dimension limit (for scaling)", min_, CRP::System));
    config->push_back(CRP("input_max", "vector.observation_max", "Upper input dimension limit (for scaling)", max_, CRP::System));
  }
  else if (role == "action")
  {
    config->push_back(CRP("input_min", "vector.action_min", "Lower input dimension limit (for scaling)", min_, CRP::System));
    config->push_back(CRP("input_max", "vector.action_max", "Upper input dimension limit (for scaling)", max_, CRP::System));
  }
  else if (role == "pair")
  {
    config->push_back(CRP("input_min", "vector.observation_min+vector.action_min", "Lower input dimension limit (for scaling)", min_, CRP::System));
    config->push_back(CRP("input_max", "vector.observation_max+vector.action_max", "Upper input dimension limit (for scaling)", max_, CRP::System));
  }
  else
  {
    config->push_back(CRP("input_min", "Lower input dimension limit (for scaling)", min_, CRP::System));
    config->push_back(CRP("input_max", "Upper input dimension limit (for scaling)", max_, CRP::System));
  }

  config->push_back(CRP("projector", "projector." + role, "Downstream projector", projector_));
}

void NormalizingProjector::configure(Configuration &config)
{
  projector_ = (Projector*) config["projector"].ptr();
  min_ = config["input_min"];
  max_ = config["input_max"];
  
  if (min_.size() != max_.size())
    throw bad_param("projector/normalizing:{input_min,input_max}");

  scaling_ = ConstantVector(min_.size(), 1.)/(max_-min_);
}

void NormalizingProjector::reconfigure(const Configuration &config)
{
}

NormalizingProjector *NormalizingProjector::clone() const
{
  NormalizingProjector *projector = new NormalizingProjector(*this);
  projector->projector_ = projector_->clone();
  return projector;
}

ProjectionPtr NormalizingProjector::project(const Vector &in) const
{
  if (in.size() != scaling_.size())
    throw bad_param("projector/pre/normalizing:{min,max,scaling}"); 

  return projector_->project((in-min_)*scaling_);
}

Matrix NormalizingProjector::jacobian(const Vector &in) const
{
  if (in.size() != scaling_.size())
    throw bad_param("projector/pre/normalizing:{min,max,scaling}"); 

  return projector_->jacobian((in-min_)*scaling_)*diagonal(scaling_);
}
