/** \file rollout.cpp
 * \brief Policy rollout agent source file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2020-01-15
 *
 * \copyright \verbatim
 * Copyright (c) 2020, Wouter Caarls
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

#include <grl/agents/rollout.h>

using namespace grl;

REGISTER_CONFIGURABLE(RolloutAgent)

void RolloutAgent::request(ConfigurationRequest *config)
{
  config->push_back(CRP("steps", "Number of control steps between updates", steps_, CRP::Configuration, 1));

  config->push_back(CRP("policy", "mapping/policy", "Control policy", policy_));
  config->push_back(CRP("predictor", "predictor", "Value function predictor", predictor_));
}

void RolloutAgent::configure(Configuration &config)
{
  steps_ = config["steps"];
  policy_ = (Policy*)config["policy"].ptr();
  predictor_ = (Predictor*)config["predictor"].ptr();
  
  transitions_.reserve(steps_);
}

void RolloutAgent::reconfigure(const Configuration &config)
{
}

void RolloutAgent::start(const Observation &obs, Action *action)
{
  time_ = 0;
  policy_->act(time_, obs, action);
  
  prev_obs_ = obs;
  prev_action_ = *action;
}

void RolloutAgent::step(double tau, const Observation &obs, double reward, Action *action)
{
  time_ += tau;
  policy_->act(time_, obs, action);
  
  addTransition(prev_obs_, prev_action_, tau, reward, obs, *action);

  prev_obs_ = obs;
  prev_action_ = *action;
}

void RolloutAgent::end(double tau, const Observation &obs, double reward)
{
  addTransition(prev_obs_, prev_action_, tau, reward, obs);
}

void RolloutAgent::addTransition(const Observation &prev_obs, const Action &prev_action, double tau, double reward, const Observation &obs, const Action &action)
{
  transitions_.push_back(new Transition(prev_obs, prev_action, tau, reward, obs, action));
  
  if (transitions_.size() == steps_)
  {
    predictor_->update(transitions_);
    transitions_.clear();
  }
}
