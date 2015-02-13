/** \file black_box.cpp
 * \brief Black box optimization agent source file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-02-13
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

#include <grl/agents/black_box.h>

using namespace grl;

REGISTER_CONFIGURABLE(BlackBoxAgent)

void BlackBoxAgent::request(ConfigurationRequest *config)
{
  config->push_back(CRP("episodes", "Number of episodes to evaluate policy", episodes_, CRP::Configuration, 1));
  
  config->push_back(CRP("optimizer", "optimizer", "Policy optimizer", optimizer_));
}

void BlackBoxAgent::configure(Configuration &config)
{
  optimizer_ = (Optimizer*)config["optimizer"].ptr();
  episodes_ = config["episodes"];
  
  reset();
}  

void BlackBoxAgent::reconfigure(const Configuration &config)
{
  if (config.has("action") && config["action"].str() == "reset")
  {
    index_ = 0;
    episode_ = -1;
    reward_ = 0;
  
    policy_ = optimizer_->request(index_);
  }
}

BlackBoxAgent *BlackBoxAgent::clone() const
{
  BlackBoxAgent *agent = new BlackBoxAgent(*this);
  agent->policy_ = policy_->clone();
  agent->optimizer_ = optimizer_->clone();
  
  return agent;
}

void BlackBoxAgent::start(const Vector &obs, Vector *action)
{
  if (++episode_ == episodes_)
  {
    optimizer_->report(index_, reward_);
    
    if (++index_ == optimizer_->size())
      index_ = 0;

    episode_ = 0;
    reward_ = 0;
    
    policy_ = optimizer_->request(index_);
  }

  policy_->act(obs, action);
  prev_obs_ = obs;
  prev_action_ = *action;
}

void BlackBoxAgent::step(const Vector &obs, double reward, Vector *action)
{
  reward_ += reward;

  policy_->act(prev_obs_, prev_action_, obs, action);
  prev_obs_ = obs;
  prev_action_ = *action;
}

void BlackBoxAgent::end(double reward)
{
  reward_ += reward;
}
