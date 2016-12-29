/** \file fixed.cpp
 * \brief Fixed-policy agent source file.
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

#include <grl/agents/fixed.h>

using namespace grl;

REGISTER_CONFIGURABLE(FixedAgent)

void FixedAgent::request(ConfigurationRequest *config)
{
  config->push_back(CRP("policy", "policy", "Control policy", policy_));
}

void FixedAgent::configure(Configuration &config)
{
  policy_ = (Policy*)config["policy"].ptr();
}

void FixedAgent::reconfigure(const Configuration &config)
{
}

FixedAgent *FixedAgent::clone() const
{
  FixedAgent *agent = new FixedAgent();
  agent->policy_ = policy_->clone();
  
  return agent;
}

TransitionType FixedAgent::start(const Vector &obs, Vector *action)
{
  time_ = 0.;
  return policy_->act(time_, obs, action);
}

TransitionType FixedAgent::step(double tau, const Vector &obs, double reward, Vector *action)
{
  time_ += tau;
  return policy_->act(time_, obs, action);
}

void FixedAgent::end(double tau, const Vector &obs, double reward)
{
}
