/** \file exclusive.cpp
 * \brief Exclusive master agent source file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-06-16
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

#include <grl/agents/exclusive.h>

using namespace grl;

REGISTER_CONFIGURABLE(ExclusiveMasterAgent)

void ExclusiveMasterAgent::request(ConfigurationRequest *config)
{
  config->push_back(CRP("gamma", "Discount rate", gamma_));
  config->push_back(CRP("predictor", "predictor", "Optional (model) predictor", predictor_, true));
  config->push_back(CRP("agent1", "agent/sub", "First subagent", agent_[0]));
  config->push_back(CRP("agent2", "agent/sub", "Second subagent", agent_[1]));
}

void ExclusiveMasterAgent::configure(Configuration &config)
{
  predictor_ = (Predictor*)config["predictor"].ptr();
  agent_[0] = (SubAgent*)config["agent1"].ptr();
  agent_[1] = (SubAgent*)config["agent2"].ptr();
}

void ExclusiveMasterAgent::reconfigure(const Configuration &config)
{
  if (config.has("action") && config["action"].str() == "reset")
    time_[0] = time_[1] = -1;
}

ExclusiveMasterAgent *ExclusiveMasterAgent::clone() const
{
  return NULL;
}

void ExclusiveMasterAgent::start(const Vector &obs, Vector *action)
{
  // Treat a terminal, non-absorbing state as absorbing for the
  // agent that was not running. The rationale is that it did not
  // exit the macro-state, which is therefore absorbing.
  if (time_[1-last_agent_] >= 0)
  {
    double stau = time_[last_agent_] - time_[1-last_agent_];
    CRAWL("Virtual absorbing state for non-running agent " << 1-last_agent_ << " (SMDP reward " << reward_ << " over " << stau << "s)");
    agent_[1-last_agent_]->end(stau, obs, reward_);
  }
  
  time_[0] = time_[1] = -1;
  
  last_agent_ = agent_[1]->confidence(obs) > agent_[0]->confidence(obs);

  CRAWL("Starting with agent " << last_agent_);

  agent_[last_agent_]->start(obs, action);
  time_[last_agent_] = 0.;
  
  reward_ = 0;
  smdp_steps_ = 0;
  prev_obs_ = obs;
  prev_action_ = *action;
}

void ExclusiveMasterAgent::step(double tau, const Vector &obs, double reward, Vector *action)
{
  double time = time_[last_agent_] + tau;
  int new_agent = agent_[1]->confidence(obs) > agent_[0]->confidence(obs);
  
  // Semi-MDP style reward. Should be per sub-agent.
  reward_ += pow(gamma_, smdp_steps_)*reward;
    
  if (new_agent == last_agent_)
  {
    agent_[last_agent_]->step(tau, obs, reward, action);
  }
  else
  {
    if (time_[new_agent] < 0)
    {
      CRAWL("Switching to agent " << new_agent << " (starting)");
      agent_[new_agent]->start(obs, action);
    }
    else
    {
      double stau = time-time_[new_agent];
      CRAWL("Switching to agent " << new_agent << " (continuing, SMDP reward " << reward_ << " over " << stau << "s)");
      agent_[new_agent]->step(stau, obs, reward_, action);
    }
      
    last_agent_ = new_agent;
    reward_ = reward;
    smdp_steps_ = 1;
  }

  time_[new_agent] = time;

  if (predictor_)
  {
    Transition t(prev_obs_, prev_action_, reward, obs, *action);
    predictor_->update(t);
  }
  prev_obs_ = obs;
  prev_action_ = *action;
}

void ExclusiveMasterAgent::end(double tau, const Vector &obs, double reward)
{
  reward_ += pow(gamma_, smdp_steps_)*reward;
  
  agent_[last_agent_]->end(tau, obs, reward);
  
  if (time_[1-last_agent_] >= 0)
  {
    CRAWL("Absorbing state for non-running agent " << 1-last_agent_ << " (SMDP reward " << reward_ << ")");
    agent_[1-last_agent_]->end(time_[last_agent_]+tau-time_[1-last_agent_], obs, reward_);
  }
  
  time_[0] = time_[1] = -1;

  if (predictor_)
  {
    Transition t(prev_obs_, prev_action_, reward, obs);
    predictor_->update(t);
  }
}
