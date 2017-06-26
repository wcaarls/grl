/** \file remapping.cpp
 * \brief Remapping agent source file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2017-06-26
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

#include <grl/agents/remapping.h>

using namespace grl;

REGISTER_CONFIGURABLE(RemappingAgent)

// *** RemappingAgent ***

void RemappingAgent::request(ConfigurationRequest *config)
{
  config->push_back(CRP("mapping", "mapping", "Maps downstream observation-action pairs onto upstream actions", mapping_));
  config->push_back(CRP("agent", "agent", "Downstream agent", agent_));
}

void RemappingAgent::configure(Configuration &config)
{
  mapping_ = (Mapping*)config["mapping"].ptr();
  agent_ = (Agent*)config["agent"].ptr();
}

void RemappingAgent::reconfigure(const Configuration &config)
{
}

void RemappingAgent::start(const Observation &obs, Action *action)
{
  Action downstream_action;
  
  agent_->start(obs, &downstream_action);
  
  mapping_->read(extend(obs.v, downstream_action.v), &action->v);
  action->type = downstream_action.type;
}

void RemappingAgent::step(double tau, const Observation &obs, double reward, Action *action)
{
  Action downstream_action;
  
  agent_->step(tau, obs, reward, &downstream_action);
  
  mapping_->read(extend(obs.v, downstream_action.v), &action->v);
  action->type = downstream_action.type;
}

void RemappingAgent::end(double tau, const Observation &obs, double reward)
{
  agent_->end(tau, obs, reward);
}
