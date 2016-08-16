/** \file leo_learn_squatting.cpp
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

#include <grl/agents/leo_learn_squatting.h>

using namespace grl;

REGISTER_CONFIGURABLE(LeoLearnSquatting)

void LeoLearnSquatting::request(ConfigurationRequest *config)
{
  config->push_back(CRP("policy_standup", "policy", "Safe standup policy", policy_standup_));
  config->push_back(CRP("policy_learn", "policy", "Learning policy", policy_learn_));
}

void LeoLearnSquatting::configure(Configuration &config)
{
  policy_standup_ = (Policy*)config["policy_standup"].ptr();
  policy_learn_ = (Policy*)config["policy_learn"].ptr();
}

void LeoLearnSquatting::reconfigure(const Configuration &config)
{
}

LeoLearnSquatting *LeoLearnSquatting::clone() const
{
  LeoLearnSquatting *agent = new LeoLearnSquatting();
  agent->policy_standup_ = policy_standup_->clone();
  agent->policy_learn_ = policy_learn_->clone();
  
  return agent;
}

void LeoLearnSquatting::start(const Vector &obs, Vector *action)
{
  time_ = 0.;
  policy_ = policy_standup_;
  policy_->act(time_, obs, action);
}

void LeoLearnSquatting::step(double tau, const Vector &obs, double reward, Vector *action)
{
  time_ += tau;

  if (policy_ == policy_standup_)
    if (trigger_.check(time_, obs))
      policy_ = policy_learn_;

  policy_->act(time_, obs, action);
}

void LeoLearnSquatting::end(double tau, const Vector &obs, double reward)
{
}
