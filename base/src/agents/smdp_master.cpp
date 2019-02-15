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

  config->push_back(CRP("predictor", "predictor", "Optional (model) predictor", predictor_, true));
  config->push_back(CRP("agent", "agent/sub", "Subagents", &agent_));
}

void SMDPMasterAgent::configure(Configuration &config)
{
  gamma_ = config["gamma"];

  predictor_ = (Predictor*)config["predictor"].ptr();
  agent_ = *(ConfigurableList*)config["agent"].ptr();
  
  if (agent_.size() < 1)
    throw bad_param("agent/master:agent");
    
  time_ = std::vector<double>(agent_.size(), -1.);
  reward_ = std::vector<double>(agent_.size(), 0.);
}

void SMDPMasterAgent::reconfigure(const Configuration &config)
{
  if (config.has("action") && config["action"].str() == "reset")
    time_ = std::vector<double>(agent_.size(), -1.);
}

void SMDPMasterAgent::start(const Observation &obs, Action *action)
{
  // Treat a terminal, non-absorbing state as absorbing for
  // agents that weren't running. The rationale is that it did not
  // exit the macro-state, which is therefore absorbing.
  for (size_t ii=0; ii < agent_.size(); ++ii)
  {
    if (time_[ii] >= 0 && time_[ii] < prev_time_)
    {
      CRAWL("Pseudo-ending agent " << ii << " with reward " << reward_[ii]);
      agent_[ii]->end(prev_time_-time_[ii], prev_obs_, reward_[ii]);
    }
    
    time_[ii] = -1;
  }

  // Let derived class decide which agents to run.
  runSubAgents(0, obs, action);

  prev_obs_ = obs;
  prev_action_ = *action;
  prev_time_ = 0;
}

void SMDPMasterAgent::step(double tau, const Observation &obs, double reward, Action *action)
{
  double curtime = prev_time_ + tau;

  // Calculate sMDP rewards for all agents.
  for (size_t ii=0; ii < agent_.size(); ++ii)
    reward_[ii] += pow(gamma_, prev_time_ - time_[ii]) * reward;
    
  // Let derived class decide which agents to run.
  runSubAgents(curtime, obs, action);
    
  if (predictor_)
  {
    Transition t(prev_obs_, prev_action_, tau, reward, obs, *action);
    predictor_->update(t);
  }
  
  prev_obs_ = obs;
  prev_action_ = *action;
  prev_time_ = curtime;
}

void SMDPMasterAgent::end(double tau, const Observation &obs, double reward)
{
  double curtime = prev_time_ + tau;

  // Give final rewards to all running agents.
  for (size_t ii=0; ii < agent_.size(); ++ii)
  {
    reward_[ii] += pow(gamma_, prev_time_ - time_[ii]) * reward;
    
    if (time_[ii] >= 0)
    {
      CRAWL("Ending agent " << ii << " with reward " << reward_[ii]);
      agent_[ii]->end(curtime-time_[ii], obs, reward_[ii]);
    }
    time_[ii] = curtime;
    reward_[ii] = 0;
  }

  if (predictor_)
  {
    Transition t(prev_obs_, prev_action_, tau, reward, obs);
    predictor_->update(t);
  }
}

double SMDPMasterAgent::runSubAgent(size_t idx, double time, const Observation &obs, Action *action)
{
  double confidence;

  if (time_[idx] < 0)
  {
    CRAWL("Starting agent " << idx);
    agent_[idx]->start(obs, action, &confidence);
  }
  else
  {
    CRAWL("Stepping agent " << idx << " with reward " << reward_[idx]);
    agent_[idx]->step(time-time_[idx], obs, reward_[idx], action, &confidence);
  }
  
  time_[idx] = time;
  reward_[idx] = 0;
  
  return confidence;
}

// *** ExclusiveMasterAgent ***

void ExclusiveMasterAgent::runSubAgents(double time, const Observation &obs, Action *action)
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

void PredicatedMasterAgent::runSubAgents(double time, const Observation &obs, Action *action)
{
  // Run agents until we find a confident one
  for (size_t ii=0; ii < agent_.size(); ++ii)
    if (runSubAgent(ii, time, obs, action) > 0.5)
      break;
}

// *** RandomMasterAgent ***

void RandomMasterAgent::runSubAgents(double time, const Observation &obs, Action *action)
{
  // Run random agent
  size_t idx = RandGen::getInteger(agent_.size());
  runSubAgent(idx, time, obs, action);
}
