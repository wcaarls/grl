/** \file predicated.cpp
 * \brief Predicated master agent source file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2016-02-03
 *
 * \copyright \verbatim
 * Copyright (c) 2016, Wouter Caarls
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

#include <grl/agents/predicated.h>

using namespace grl;

REGISTER_CONFIGURABLE(PredicatedMasterAgent)

void PredicatedMasterAgent::request(ConfigurationRequest *config)
{
  config->push_back(CRP("gamma", "Discount rate", gamma_));
  config->push_back(CRP("control_step", "double.control_step", "Characteristic step time on which gamma is defined", tau_, CRP::System));
  
  config->push_back(CRP("predictor", "predictor", "Optional (model) predictor", predictor_, true));
  config->push_back(CRP("agent1", "agent/sub", "First subagent", agent_[0]));
  config->push_back(CRP("agent2", "agent/sub", "Second subagent", agent_[1]));
}

void PredicatedMasterAgent::configure(Configuration &config)
{
  gamma_ = config["gamma"];
  tau_ = config["control_step"];

  predictor_ = (Predictor*)config["predictor"].ptr();
  agent_[0] = (SubAgent*)config["agent1"].ptr();
  agent_[1] = (SubAgent*)config["agent2"].ptr();
}

void PredicatedMasterAgent::reconfigure(const Configuration &config)
{
  if (config.has("action") && config["action"].str() == "reset")
    time_[0] = time_[1] = -std::numeric_limits<double>::epsilon();
}

PredicatedMasterAgent *PredicatedMasterAgent::clone() const
{
  return NULL;
}

void PredicatedMasterAgent::start(const Vector &obs, Vector *action)
{
  double time = time_[0];

  for (size_t ii=0; ii < agent_.size(); ++ii)
  {
    if (time_[ii] >= 0 && time_[ii] < time)
      agent_[ii]->step(time-time_[ii], prev_obs_, reward_[ii], action);
    
    time_[ii] = -std::numeric_limits<double>::epsilon();
    reward_[ii] = 0;
  }

  for (size_t ii=0; ii < agent_.size(); ++ii)
  {
    double confidence;
    agent_[ii]->start(obs, action, &confidence);
    time_[ii] = 0.;
    if (confidence > 0.5)
      break;
  }

  prev_obs_ = obs;
  prev_action_ = *action;
}

void PredicatedMasterAgent::step(double tau, const Vector &obs, double reward, Vector *action)
{
  double prevtime = time_[0], curtime = prevtime + tau;

  for (size_t ii=0; ii < agent_.size(); ++ii)
  {
    double confidence;
    reward_[ii] += pow(pow(gamma_, 1./tau_), prevtime - time_[ii]) * reward;
    
    agent_[ii]->step(curtime-time_[ii], obs, reward_[ii], action, &confidence);
    time_[ii] = curtime;
    reward_[ii] = 0;
    
    if (confidence > 0.5)
      for (++ii; ii < agent_.size(); ++ii)
        reward_[ii] += pow(pow(gamma_, 1./tau_), prevtime - time_[ii]) * reward;
  }
  
  if (predictor_)
  {
    Transition t(prev_obs_, prev_action_, reward, obs, *action);
    predictor_->update(t);
  }
  
  prev_obs_ = obs;
  prev_action_ = *action;
}

void PredicatedMasterAgent::end(double tau, const Vector &obs, double reward)
{
  double prevtime = time_[0], curtime = prevtime + tau;

  for (size_t ii=0; ii < agent_.size(); ++ii)
  {
    reward_[ii] += pow(pow(gamma_, 1./tau_), prevtime - time_[ii]) * reward;
    
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
