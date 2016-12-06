/** \file smdp.cpp
 * \brief sMDP master agent source file.
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

#include <grl/agents/smdp_master.h>

using namespace grl;

REGISTER_CONFIGURABLE(ExclusiveMasterAgent)
REGISTER_CONFIGURABLE(PredicatedMasterAgent)
REGISTER_CONFIGURABLE(RandomMasterAgent)

// *** SMDPMasterAgent ***

void SMDPMasterAgent::request(ConfigurationRequest *config)
{
  config->push_back(CRP("gamma", "Discount rate", gamma_));
  config->push_back(CRP("control_step", "double.control_step", "Characteristic step time on which gamma is defined", tau_, CRP::System));

  config->push_back(CRP("predictor", "predictor", "Optional (model) predictor", predictor_, true));
  config->push_back(CRP("agent1", "agent/sub", "First subagent", agent_[0]));
  config->push_back(CRP("agent2", "agent/sub", "Second subagent", agent_[1]));
}

void SMDPMasterAgent::configure(Configuration &config)
{
  gamma_ = config["gamma"];
  tau_ = config["control_step"];

  predictor_ = (Predictor*)config["predictor"].ptr();
  agent_[0] = (SubAgent*)config["agent1"].ptr();
  agent_[1] = (SubAgent*)config["agent2"].ptr();
}

void SMDPMasterAgent::reconfigure(const Configuration &config)
{
  if (config.has("action") && config["action"].str() == "reset")
    time_[0] = time_[1] = -1;
}

SMDPMasterAgent *SMDPMasterAgent::clone() const
{
  return NULL;
}


void SMDPMasterAgent::start(const Vector &obs, Vector *action)
{
  // Treat a terminal, non-absorbing state as absorbing for
  // agents that weren't running. The rationale is that it did not
  // exit the macro-state, which is therefore absorbing.
  for (size_t ii=0; ii < agent_.size(); ++ii)
  {
    if (time_[ii] >= 0 && time_[ii] < prev_time_)
      agent_[ii]->end(prev_time_-time_[ii], prev_obs_, reward_[ii]);
    
    time_[ii] = -1;
  }

  // Let derived class decide which agents to run.
  runSubAgents(0, obs, action);

  prev_obs_ = obs;
  prev_action_ = *action;
  prev_time_ = 0;
}

void SMDPMasterAgent::step(double tau, const Vector &obs, double reward, Vector *action)
{
  double curtime = prev_time_ + tau;

  // Calculate sMDP rewards for all agents.
  for (size_t ii=0; ii < agent_.size(); ++ii)
    reward_[ii] += pow(pow(gamma_, 1./tau_), prev_time_ - time_[ii]) * reward;
    
  // Let derived class decide which agents to run.
  runSubAgents(curtime, obs, action);
    
  if (predictor_)
  {
    Transition t(prev_obs_, prev_action_, reward, obs, *action);
    predictor_->update(t);
  }
  
  prev_obs_ = obs;
  prev_action_ = *action;
  prev_time_ = curtime;
}

void SMDPMasterAgent::end(double tau, const Vector &obs, double reward)
{
  double curtime = prev_time_ + tau;

  // Give final rewards to all running agents.
  for (size_t ii=0; ii < agent_.size(); ++ii)
  {
    reward_[ii] += pow(pow(gamma_, 1./tau_), prev_time_ - time_[ii]) * reward;
    
    if (time_[ii] >= 0)
      agent_[ii]->end(curtime-time_[ii], obs, reward_[ii]);
    time_[ii] = curtime;
    reward_[ii] = 0;
  }

  if (predictor_)
  {
    Transition t(prev_obs_, prev_action_, reward, obs);
    predictor_->update(t);
  }
}

double SMDPMasterAgent::runSubAgent(size_t idx, double time, const Vector &obs, Vector *action)
{
  double confidence;

  if (time_[idx] < 0)
    agent_[idx]->start(obs, action, &confidence);
  else
    agent_[idx]->step(time-time_[idx], obs, reward_[idx], action, &confidence);
  
  time_[idx] = time;
  reward_[idx] = 0;
  
  return confidence;
}

// *** ExclusiveMasterAgent ***

void ExclusiveMasterAgent::runSubAgents(double time, const Vector &obs, Vector *action)
{
  // Find most confident agent
  double maxconf = agent_[0]->confidence(obs);
  size_t maxconfa = 0;
  
  for (size_t ii=1; ii < agent_.size(); ++ii)
  {
    double confidence = agent_[ii]->confidence(obs);
    if (confidence > maxconf)
    {
      maxconf = confidence;
      maxconfa = ii;
    }
  }
  
  runSubAgent(maxconfa, time, obs, action);
}

// *** PredicatedMasterAgent ***

void PredicatedMasterAgent::runSubAgents(double time, const Vector &obs, Vector *action)
{
  // Run agents until we find a confident one
  for (size_t ii=0; ii < agent_.size(); ++ii)
    if (runSubAgent(ii, time, obs, action) > 0.5)
      break;
}

// *** RandomMasterAgent ***

void RandomMasterAgent::runSubAgents(double time, const Vector &obs, Vector *action)
{
  // Run agents until we find a confident one
  size_t idx = RandGen::getInteger(agent_.size());
  runSubAgent(idx, time, obs, action);
}
