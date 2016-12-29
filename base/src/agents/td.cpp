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

  config->push_back(CRP("pub_transition_type", "signal", "Publisher of the transition type", pub_transition_type_, true));
}

void TDAgent::configure(Configuration &config)
{
  policy_ = (Policy*)config["policy"].ptr();
  predictor_ = (Predictor*)config["predictor"].ptr();

  pub_transition_type_ = (VectorSignal*)config["pub_transition_type"].ptr();
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
  TDAgentState *state = agent_state_.instance();

  predictor_->finalize();
  
  state->time = 0;
  TransitionType tt = policy_->act(state->time, obs, action);
  
  state->prev_obs = obs;
  state->prev_action = *action;
  if (pub_transition_type_)
    pub_transition_type_->set(VectorConstructor((double)tt));
}

void TDAgent::step(double tau, const Vector &obs, double reward, Vector *action)
{
  TDAgentState *state = agent_state_.instance();

  state->time += tau;
  TransitionType tt = policy_->act(state->time, obs, action);
  
  predictor_->update(Transition(state->prev_obs, state->prev_action, reward, obs, *action, tt));

  state->prev_obs = obs;
  state->prev_action = *action;
  if (pub_transition_type_)
    pub_transition_type_->set(VectorConstructor((double)tt));
}

void TDAgent::end(double tau, const Vector &obs, double reward)
{
  TDAgentState *state = agent_state_.instance();

  predictor_->update(Transition(state->prev_obs, state->prev_action, reward, obs));
}
