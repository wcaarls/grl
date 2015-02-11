/** \file td.cpp
 * \brief Temporal difference agent source file.
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

#include <grl/agents/td.h>

using namespace grl;

REGISTER_CONFIGURABLE(TDAgent)

void TDAgent::request(ConfigurationRequest *config)
{
  config->push_back(CRP("policy", "policy", "Control policy", policy_));
  config->push_back(CRP("predictor", "predictor", "Value function predictor", predictor_));
}

void TDAgent::configure(Configuration &config)
{
  policy_ = (Policy*)config["policy"].ptr();
  predictor_ = (Predictor*)config["predictor"].ptr();
}

void TDAgent::reconfigure(const Configuration &config)
{
}

TDAgent *TDAgent::clone() const
{
  TDAgent *agent = new TDAgent();
  agent->policy_ = policy_->clone();
  agent->predictor_= predictor_->clone();
  
  return agent;
}

void TDAgent::start(const Vector &obs, Vector *action)
{
  predictor_->finalize();
  policy_->act(obs, action);
  
  prev_obs_ = obs;
  prev_action_ = *action;
}

void TDAgent::step(const Vector &obs, double reward, Vector *action)
{
  policy_->act(prev_obs_, prev_action_, obs, action);
  predictor_->update(Transition(prev_obs_, prev_action_, reward, obs, *action));

  prev_obs_ = obs;
  prev_action_ = *action;  
}

void TDAgent::end(double reward)
{
  predictor_->update(Transition(prev_obs_, prev_action_, reward));
}
