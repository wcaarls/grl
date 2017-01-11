/** \file leo_td.cpp
 * \brief Leo temporal difference agent source file.
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

#include <grl/agents/leo_td.h>

using namespace grl;

REGISTER_CONFIGURABLE(LeoTDAgent)

void LeoTDAgent::request(ConfigurationRequest *config)
{
  TDAgent::request(config);
  config->push_back(CRP("pub_transition_type", "signal/vector", "Publisher of the transition type", pub_transition_type_, true));
}

void LeoTDAgent::configure(Configuration &config)
{
  TDAgent::configure(config);
  pub_transition_type_ = (VectorSignal*)config["pub_transition_type"].ptr();
}

void LeoTDAgent::reconfigure(const Configuration &config)
{
  TDAgent::reconfigure(config);
}

void LeoTDAgent::start(const Observation &obs, Action *action)
{
  TDAgent::start(obs, action);

  if (pub_transition_type_)
    pub_transition_type_->set(VectorConstructor((double)action->type));
}

void LeoTDAgent::step(double tau, const Observation &obs, double reward, Action *action)
{
  TDAgent::step(tau, obs, reward, action);

  if (pub_transition_type_)
    pub_transition_type_->set(VectorConstructor((double)action->type));
}
