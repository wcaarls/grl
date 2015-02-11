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

#include <grl/agents/dyna.h>

using namespace grl;

REGISTER_CONFIGURABLE(DynaAgent)

void DynaAgent::request(ConfigurationRequest *config)
{
  config->push_back(CRP("planning_steps", "Number of planning steps per control step", planning_steps_, CRP::Configuration, 0));
  config->push_back(CRP("wrapping", "Wrapping boundaries", wrapping_));
  config->push_back(CRP("observation_min", "Lower limit on observations", observation_min_, CRP::System));
  config->push_back(CRP("observation_max", "Upper limit on observations", observation_max_, CRP::System));

  config->push_back(CRP("policy", "policy", "Control policy", policy_));
  config->push_back(CRP("predictor", "predictor", "Value function predictor", predictor_));
  config->push_back(CRP("model_agent", "agent", "Agent used for planning episodes", model_agent_));
  config->push_back(CRP("model_projector", "projector", "Projector for transition model (should match representation)", model_projector_));
  config->push_back(CRP("model_representation", "representation", "Representation for transition model", model_representation_));
}

void DynaAgent::configure(Configuration &config)
{
  policy_ = (Policy*)config["policy"].ptr();
  predictor_ = (Predictor*)config["predictor"].ptr();

  planning_steps_ = config["planning_steps"];
  
  model_agent_ = (Agent*)config["model_agent"].ptr();
  model_projector_ = (Projector*)config["model_projector"].ptr();
  model_representation_ = (Representation*)config["model_representation"].ptr();
  
  wrapping_ = config["wrapping"];
  
  observation_min_ = config["observation_min"];
  observation_max_ = config["observation_max"];
}

void DynaAgent::reconfigure(const Configuration &config)
{
}

DynaAgent *DynaAgent::clone() const
{
  DynaAgent *agent = new DynaAgent(*this);
  
  agent->policy_ = policy_->clone();
  agent->predictor_= predictor_->clone();
  agent->model_agent_= model_agent_->clone();
  agent->model_projector_= model_projector_->clone();
  agent->model_representation_= model_representation_->clone();
  
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
  
  learnModel(t);
  runModel();

  prev_obs_ = obs;
  prev_action_ = *action;  
}

void DynaAgent::end(double reward)
{
  Transition t(prev_obs_, prev_action_, reward);
  predictor_->update(t);
  
  t.obs = prev_obs_;
  t.action = prev_action_;
  learnModel(t);
  runModel();
}

void DynaAgent::learnModel(const Transition &t)
{
  Vector target;
  
  if (!t.obs.empty())
  {
    target = t.obs-t.prev_obs;
    
    for (size_t ii=0; ii < target.size(); ++ii)
      if (wrapping_[ii])
      {
        if (target[ii] > 0.5*wrapping_[ii])
          target[ii] -= wrapping_[ii];
        else if (target[ii] < -0.5*wrapping_[ii])
          target[ii] += wrapping_[ii];
      }
    
    target.push_back(t.reward);
    target.push_back(0);
  }
  else
  {
    // Absorbing state
    target.resize(t.prev_obs.size(), 0.);
    target.push_back(t.reward);
    target.push_back(1);
  }
  
  ProjectionPtr p = model_projector_->project(extend(t.prev_obs, t.prev_action));
  model_representation_->write(p, target);
}

void DynaAgent::stepModel(const Vector &obs, const Vector &action, Vector *next, double *reward, int *terminal)
{
  ProjectionPtr p = model_projector_->project(extend(obs, action)); 
  
  if (!p)
  {
    next->clear();
    return;
  }
 
  model_representation_->read(p, next);
  
  bool valid = !next->empty();
  for (size_t ii=0; ii < obs.size() && valid; ++ii)
  {
    (*next)[ii] += obs[ii];
    
    if (wrapping_[ii])
      (*next)[ii] = fmod(fmod((*next)[ii], wrapping_[ii]) + wrapping_[ii], wrapping_[ii]);
    
    if ((*next)[ii] < observation_min_[ii] || (*next)[ii] > observation_max_[ii])
      valid = false;
  }

  // Guard against failed model prediction
  if (valid)
  {
    *reward = (*next)[next->size()-2];  
    *terminal = (int)(*next)[next->size()-1];
    next->resize(next->size()-2);
  }
  else
    next->clear();
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
      obs = start_obs_;
      model_agent_->start(obs, &action);
    }
      
    Vector next;
    double reward;
  
    stepModel(obs, action, &next, &reward, &terminal);
    
    obs = next;
        
    // Guard against failed model prediction    
    if (!obs.empty())
    {
      if (terminal)
        model_agent_->end(reward);
      else
        model_agent_->step(obs, reward, &action);
    }
    else
      terminal = 1;
      
    // Break episodes after a while
    if (steps++ == 100)
    {
      steps = 0;
      terminal = 1;
    }
  }
}
