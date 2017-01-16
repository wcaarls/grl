/** \file displacement.cpp
 * \brief Policy displacement mapping source file. 
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2017-01-13
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

#include <grl/mappings/displacement.h>

using namespace grl;

REGISTER_CONFIGURABLE(DisplacementMapping)

void DisplacementMapping::request(ConfigurationRequest *config)
{
  config->push_back(CRP("policy", "mapping/policy", "Policy for which displacement is calculated", policy_));
  config->push_back(CRP("model", "observation_model", "Observation model on which policy acts", model_));
}

void DisplacementMapping::configure(Configuration &config)
{
  policy_ = (Policy*)config["policy"].ptr();
  model_ = (ObservationModel*)config["model"].ptr();
}

void DisplacementMapping::reconfigure(const Configuration &config)
{
}

double DisplacementMapping::read(const Vector &in, Vector *result) const
{
  Observation obs = in;
  Action action;
  
  policy_->act(obs, &action);
  
  Observation next;
  double reward;
  int terminal;
  model_->step(obs, action, &next, &reward, &terminal);
  
  *result = next.v - obs.v;
  
  return (*result)[0];
}
