/** \file leo_fixed.cpp
 * \brief Leo fixed agent source file.
 *
 * \author    Ivan Koryakovskiy <i.koryakovskiy@tudelft.nl>
 * \date      2016-01-01
 *
 * \copyright \verbatim
 * Copyright (c) 2016, Ivan Koryakovskiy
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

#include <grl/agents/leo_fixed.h>

using namespace grl;

REGISTER_CONFIGURABLE(LeoFixedAgent)

void LeoFixedAgent::request(ConfigurationRequest *config)
{
  FixedAgent::request(config);
  config->push_back(CRP("pub_transition_type", "signal/vector", "Publisher of the transition type", pub_transition_type_, true));
}

void LeoFixedAgent::configure(Configuration &config)
{
  FixedAgent::configure(config);
  pub_transition_type_ = (VectorSignal*)config["pub_transition_type"].ptr();
}

void LeoFixedAgent::reconfigure(const Configuration &config)
{
  FixedAgent::reconfigure(config);
}

void LeoFixedAgent::start(const Observation &obs, Action *action)
{
  FixedAgent::start(obs, action);

  if (pub_transition_type_)
    pub_transition_type_->set(VectorConstructor((double)action->type));
}

void LeoFixedAgent::step(double tau, const Observation &obs, double reward, Action *action)
{
  FixedAgent::step(tau, obs, reward, action);

  if (pub_transition_type_)
    pub_transition_type_->set(VectorConstructor((double)action->type));
}
