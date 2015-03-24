/** \file dyna.cpp
 * \brief Dyna agent source file.
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

#include <iomanip>

#include <grl/agents/dyna.h>

using namespace grl;

REGISTER_CONFIGURABLE(DynaAgent)

void DynaAgent::request(ConfigurationRequest *config)
{
  config->push_back(CRP("planning_steps", "Number of planning steps per control step", planning_steps_, CRP::Configuration, 0));

  config->push_back(CRP("policy", "policy", "Control policy", policy_));
  config->push_back(CRP("predictor", "predictor", "Value function predictor", predictor_));
  config->push_back(CRP("model", "observation_model", "Observation model used for planning", model_));
  config->push_back(CRP("model_predictor", "predictor/model", "Model predictor", model_predictor_, true));
  config->push_back(CRP("model_agent", "agent", "Agent used for planning episodes", model_agent_));
  
  config->push_back(CRP("state", "state", "Current observed state of planning", CRP::Provided));  
}

void DynaAgent::configure(Configuration &config)
{
  policy_ = (Policy*)config["policy"].ptr();
  predictor_ = (Predictor*)config["predictor"].ptr();

  planning_steps_ = config["planning_steps"];
  
  model_ = (ObservationModel*)config["model"].ptr();
  model_predictor_ = (ModelPredictor*)config["model_predictor"].ptr();
  model_agent_ = (Agent*)config["model_agent"].ptr();

  state_ = new State();
  
  config.set("state", state_);
}

void DynaAgent::reconfigure(const Configuration &config)
{
  if (config.has("action") && config["action"].str() == "reset")
    planned_steps_ = 0;
}

DynaAgent *DynaAgent::clone() const
{
  DynaAgent *agent = new DynaAgent(*this);
  
  agent->policy_ = policy_->clone();
  agent->predictor_= predictor_->clone();
  agent->model_ = model_->clone();
  agent->model_predictor_= model_predictor_->clone();
  agent->model_agent_= model_agent_->clone();
  
  return agent;
}

void DynaAgent::start(const Vector &obs, Vector *action)
{
  predictor_->finalize();
  policy_->act(obs, action);
  
  prev_obs_ = obs;
  prev_action_ = *action;
  start_obs_ = obs;
}

void DynaAgent::step(const Vector &obs, double reward, Vector *action)
{
  policy_->act(prev_obs_, prev_action_, obs, action);
  
  Transition t(prev_obs_, prev_action_, reward, obs, *action);
  predictor_->update(t);
  if (model_predictor_)
    model_predictor_->update(t);
  
  runModel();

  prev_obs_ = obs;
  prev_action_ = *action;  
}

void DynaAgent::end(double reward)
{
  Transition t(prev_obs_, prev_action_, reward);
  predictor_->update(t);
  if (model_predictor_)
    model_predictor_->update(t);
  
  runModel();
}

void DynaAgent::report(std::ostream &os) const
{
  os << std::setw(15) << planned_steps_ << std::setw(15) << planning_reward_;
  planning_reward_ = 0;
}

void DynaAgent::runModel()
{
  Vector obs, action;
  int terminal=1;
  size_t steps=0;

  for (size_t ii=0; ii < planning_steps_; ++ii)
  {
    if (terminal)
    {
      steps = 0;
      obs = start_obs_;
      state_->set(obs);
      model_agent_->start(obs, &action);
    }
      
    Vector next;
    double reward;
  
    model_->step(obs, action, &next, &reward, &terminal);
    
    obs = next;
    state_->set(obs);
        
    // Guard against failed model prediction    
    if (!obs.empty())
    {
      planned_steps_++;
    
      if (terminal == 2)
        model_agent_->end(reward);
      else
        model_agent_->step(obs, reward, &action);
        
      planning_reward_ += reward;
    }
    else
      terminal = 1;
      
    // Break episodes after a while
    if (steps++ == 100)
      terminal = 1;
  }
}
