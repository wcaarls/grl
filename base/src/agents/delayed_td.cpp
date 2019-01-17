/** \file delayed_td.cpp
 * \brief Delayed temporal difference agent source file.
 *
 * \author    Ivan Koryakovskiy <i.koryakovskiy@google.com>
 * \date      2017-02-12
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

#include <grl/agents/delayed_td.h>

using namespace grl;

REGISTER_CONFIGURABLE(DelayedTDAgent)

void DelayedTDAgent::request(ConfigurationRequest *config)
{
  config->push_back(CRP("policy", "mapping/policy", "Control policy", policy_));
  config->push_back(CRP("predictor", "predictor", "Value function predictor", predictor_));
  config->push_back(CRP("control_delay", "Relative control delay: 0 (no delay) - 1 (one timestep delay) ", (double)control_delay_, CRP::Configuration, 0.0, 1.0));
}

void DelayedTDAgent::configure(Configuration &config)
{
  policy_ = (Policy*)config["policy"].ptr();
  predictor_ = (Predictor*)config["predictor"].ptr();
  control_delay_ = config["control_delay"];
}

void DelayedTDAgent::reconfigure(const Configuration &config)
{
}

void DelayedTDAgent::start(const Observation &obs, Action *action)
{
  DelayedTDAgentState *state = agent_state_.instance();

  predictor_->finalize();

  state->time = 0;
  policy_->act(state->time, obs, action);
  
  state->prev_obs = obs;
  state->prev_action = *action;
  state->prev_prev_action = ConstantVector(action->size(), 0);
  state->prev_prev_action.type = atGreedy;
}

void DelayedTDAgent::step(double tau, const Observation &obs, double reward, Action *action)
{
  DelayedTDAgentState *state = agent_state_.instance();

  state->time += tau;
  policy_->act(state->time, obs, action);

  Action prev_action_hat = combine(state->prev_prev_action, state->prev_action);
  Action action_hat = combine(state->prev_action, *action);

  predictor_->update(Transition(state->prev_obs, prev_action_hat, tau, reward, obs, action_hat));

  state->prev_obs = obs;
  state->prev_prev_action = state->prev_action;
  state->prev_action = *action;
}

void DelayedTDAgent::end(double tau, const Observation &obs, double reward)
{
  DelayedTDAgentState *state = agent_state_.instance();

  Action prev_action_hat = combine(state->prev_prev_action, state->prev_action);
  predictor_->update(Transition(state->prev_obs, prev_action_hat, tau, reward, obs));
}

Action DelayedTDAgent::combine(const Action &a0, const Action &a1) const
{
  if (control_delay_ != 0 && control_delay_ != 1)
  {
    Action a_hat;
    a_hat.v = a0.v*control_delay_ + a1.v*(1-control_delay_);
    a_hat.type = (a0.type == atGreedy && a1.type == atGreedy) ? atGreedy : atExploratory;
    return a_hat;
  }
  else if (control_delay_ == 1)
    return a0;
  else
    return a1;
}
