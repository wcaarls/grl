/** \file leo_squatting_agent.cpp
 * \brief State-machine agent source file which performs squatting on Leo.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-01-22
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

#include <grl/agents/leo_squatting_agent.h>

using namespace grl;

REGISTER_CONFIGURABLE(LeoSquattingAgent)

void LeoSquattingAgent::request(ConfigurationRequest *config)
{
  config->push_back(CRP("agent_standup", "agent", "Safe standup agent", agent_standup_));
  config->push_back(CRP("agent_learn", "agent", "Learning agent", agent_learn_));
}

void LeoSquattingAgent::configure(Configuration &config)
{
  agent_standup_ = (Agent*)config["agent_standup"].ptr();
  agent_learn_ = (Agent*)config["agent_learn"].ptr();
}

void LeoSquattingAgent::reconfigure(const Configuration &config)
{
}

LeoSquattingAgent *LeoSquattingAgent::clone() const
{
  LeoSquattingAgent *agent = new LeoSquattingAgent();
  agent->agent_standup_ = agent_standup_->clone();
  agent->agent_learn_ = agent_learn_->clone();
  return agent;
}

TransitionType LeoSquattingAgent::start(const Vector &obs, Vector *action)
{
  time_ = 0.;
  agent_ = agent_standup_;
  return agent_->start(obs, action);
}

TransitionType LeoSquattingAgent::step(double tau, const Vector &obs, double reward, Vector *action)
{
  time_ += tau;

  if (agent_ == agent_standup_)
    if (trigger_.check(time_, obs))
    {
      agent_ = agent_learn_;
      return agent_->start(obs, action);
    }

  return agent_->step(tau, obs, reward, action);
}

void LeoSquattingAgent::end(double tau, const Vector &obs, double reward)
{
  agent_standup_->end(tau, obs, reward);
  if (agent_ == agent_learn_)
    agent_learn_->end(tau, obs, reward);
}
