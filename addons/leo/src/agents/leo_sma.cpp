/** \file leo_sma.cpp
 * \brief State-machine agent source file for Leo
 *
 * \author    Ivan Koryakovskiy <i.koryakovskiy@tudelft.nl>
 * \date      2016-01-01
 *
 * \copyright \verbatim
 * Copyright (c) 2016, Ivan Koryakovskiy
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

#include <grl/agents/leo_sma.h>
#include <leo.h>

using namespace grl;

REGISTER_CONFIGURABLE(LeoStateMachineAgent)

void LeoStateMachineAgent::request(ConfigurationRequest *config)
{
  config->push_back(CRP("agent_prepare", "agent", "Prepare agent", agent_prepare_, false));
  config->push_back(CRP("agent_standup", "agent", "Safe standup agent", agent_standup_, false));
  config->push_back(CRP("agent_starter", "agent", "Starting agent", agent_starter_, true));
  config->push_back(CRP("agent_main", "agent", "Main agent", agent_main_, false));

  config->push_back(CRP("upright_trigger", "trigger", "Trigger which finishes stand-up phase and triggers preparation agent", upright_trigger_, false));
  config->push_back(CRP("fc_trigger", "trigger", "Trigger which checks for foot contact to ensure that robot is prepared to walk", foot_contact_trigger_, false));
  config->push_back(CRP("starter_trigger", "trigger", "Trigger which initiates a preprogrammed walking at the beginning", starter_trigger_, true));
  config->push_back(CRP("sub_ic_signal", "signal/vector", "Subscriber to the contact signal", sub_ic_signal_, true));
}

void LeoStateMachineAgent::configure(Configuration &config)
{
  agent_prepare_ = (Agent*)config["agent_prepare"].ptr();
  agent_standup_ = (Agent*)config["agent_standup"].ptr();
  agent_starter_ = (Agent*)config["agent_starter"].ptr();
  agent_main_ = (Agent*)config["agent_main"].ptr();

  upright_trigger_ = (Trigger*)config["upright_trigger"].ptr();
  foot_contact_trigger_ = (Trigger*)config["fc_trigger"].ptr();
  starter_trigger_ = (Trigger*)config["starter_trigger"].ptr();
  sub_ic_signal_ = (VectorSignal*)config["sub_ic_signal"].ptr();
}

void LeoStateMachineAgent::reconfigure(const Configuration &config)
{
}

void LeoStateMachineAgent::start(const Observation &obs, Action *action)
{
  time_ = 0.;
  if (!agent_)
    agent_ = agent_prepare_;
  else
    agent_ = agent_standup_;
  agent_->start(obs, action);
}

void LeoStateMachineAgent::step(double tau, const Observation &obs, double reward, Action *action)
{
  time_ += tau;

  if (agent_ == agent_prepare_)
  {
    if (sub_ic_signal_)
    {
      Vector signal = sub_ic_signal_->get();
      Vector fc = VectorConstructor(((int)signal[0] & lstGroundContact) != 0);
      if (foot_contact_trigger_->check(time_, fc))
      {
        if (agent_starter_ && starter_trigger_ && !starter_trigger_->check(time_, Vector()))
        {
          agent_ = agent_starter_;
          INFO("Starter!");
        }
        else
        {
          agent_ = agent_main_;
          INFO("Main direct!");
        }
        agent_->start(obs, action);
        return;
      }
    }
  }

  if (agent_ == agent_starter_)
    if (starter_trigger_->check(time_, Vector()))
    {
      agent_starter_->end(tau, obs, reward);
      agent_ = agent_main_;
      agent_->start(obs, action);
      INFO("Main!");
      return;
    }

  if (agent_ == agent_standup_)
    if (upright_trigger_->check(time_, obs))
    {
      agent_standup_->end(tau, obs, reward);
      agent_ = agent_prepare_;
      agent_->start(obs, action);
      INFO("Upright!");
      return;
    }

  agent_->step(tau, obs, reward, action);
}

void LeoStateMachineAgent::end(double tau, const Observation &obs, double reward)
{
  agent_->end(tau, obs, reward);
}
