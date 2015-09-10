/** \file sequential.cpp
 * \brief Sequential master agent source file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-09-10
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

#include <grl/agents/sequential.h>

using namespace grl;

REGISTER_CONFIGURABLE(SequentialMasterAgent)

void SequentialMasterAgent::request(ConfigurationRequest *config)
{
  config->push_back(CRP("agent1", "agent", "First subagent, providing the suggested action", agent_[0]));
  config->push_back(CRP("agent2", "agent", "Second subagent, providing the final action", agent_[1]));
}

void SequentialMasterAgent::configure(Configuration &config)
{
  agent_[0] = (SubAgent*)config["agent1"].ptr();
  agent_[1] = (SubAgent*)config["agent2"].ptr();
}

void SequentialMasterAgent::reconfigure(const Configuration &config)
{
}

SequentialMasterAgent *SequentialMasterAgent::clone() const
{
  return NULL;
}

void SequentialMasterAgent::start(const Vector &obs, Vector *action)
{
  agent_[0]->start(obs, action);
  agent_[1]->start(obs, action);
}

void SequentialMasterAgent::step(double tau, const Vector &obs, double reward, Vector *action)
{
  agent_[0]->step(tau, obs, reward, action);
  agent_[1]->step(tau, obs, reward, action);
}

void SequentialMasterAgent::end(double tau, double reward)
{
  agent_[0]->end(tau, reward);
  agent_[1]->end(tau, reward);
}
