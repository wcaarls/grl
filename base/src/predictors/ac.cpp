/** \file ac.cpp
 * \brief Actor-critic predictor source file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-02-16
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

#include <grl/predictors/ac.h>

using namespace grl;

REGISTER_CONFIGURABLE(ActionACPredictor)
REGISTER_CONFIGURABLE(ExpandedActionACPredictor)
REGISTER_CONFIGURABLE(ProbabilityACPredictor)

void ActionACPredictor::request(ConfigurationRequest *config)
{
  Predictor::request(config);

  config->push_back(CRP("alpha", "Critic learning rate", alpha_));

  config->push_back(CRP("update_method", "Actor update method", update_method_, CRP::Configuration, {"proportional", "cacla"}));
  config->push_back(CRP("step_limit", "Actor exploration step limit", step_limit_));

  config->push_back(CRP("projector", "projector.observation", "Projects observations onto actor representation space", projector_));
  config->push_back(CRP("representation", "representation.action", "Action representation", representation_));
  config->push_back(CRP("critic", "predictor/critic", "Critic predictor", critic_));
}

void ActionACPredictor::configure(Configuration &config)
{
  Predictor::configure(config);

  projector_ = (Projector*)config["projector"].ptr();
  representation_ = (Representation*)config["representation"].ptr();
  critic_ = (CriticPredictor*)config["critic"].ptr();
  
  alpha_ = config["alpha"];
  
  update_method_ = config["update_method"].str();
  step_limit_ = config["step_limit"].v();
}

void ActionACPredictor::reconfigure(const Configuration &config)
{
  Predictor::reconfigure(config);

  if (config.has("action") && config["action"].str() == "reset")
    finalize();
}

void ActionACPredictor::update(const Transition &transition)
{
  Predictor::update(transition);
  
  if (step_limit_.size() && step_limit_.size() != transition.prev_action.size())
  {
    if (step_limit_.size() == 1)
      step_limit_ = ConstantVector(transition.prev_action.size(), step_limit_[0]);
    else
      throw bad_param("predictor/ac:step_limit");
  }

  ProjectionPtr ap = projector_->project(transition.prev_obs);
  
  Vector u;
  representation_->target()->read(ap, &u);
  
  if (!u.size())
    u = ConstantVector(transition.prev_action.size(), 0.);
  
  double critique = critic_->criticize(transition, u);
  
  if (update_method_[0] == 'p' || critique > 0)
  {
    Vector delta = transition.prev_action.v - u;
    
    if (update_method_[0] == 'p')
      delta = critique * delta;
    
    if (step_limit_.size())
      for (size_t ii=0; ii < delta.size(); ++ii)
        delta[ii] = fmin(fmax(delta[ii], -step_limit_[ii]), step_limit_[ii]);

    Vector target_u = u + delta;

    representation_->write(ap, target_u, alpha_);
    representation_->finalize();
  }
}

void ActionACPredictor::update(const std::vector<const Transition*> &transitions)
{
  if (step_limit_.size() && step_limit_.size() != transitions[0]->prev_action.size())
  {
    if (step_limit_.size() == 1)
      step_limit_ = ConstantVector(transitions[0]->prev_action.size(), step_limit_[0]);
    else
      throw bad_param("predictor/ac:step_limit");
  }
    
  Matrix u;
  representation_->target()->batchRead(transitions.size());
  for (size_t ii=0; ii < transitions.size(); ++ii)
    representation_->target()->enqueue(projector_->project(transitions[ii]->prev_obs));
  representation_->target()->read(&u);

  std::vector<Action> action_objs(transitions.size());
  std::vector<const Action*> actions(transitions.size());
  for (size_t ii=0; ii < transitions.size(); ++ii)
  {
    action_objs[ii] = u.row(ii).array();
    actions[ii] = &action_objs[ii];
  }
  
  LargeVector critique = critic_->criticize(transitions, actions);
  
  size_t us=0;
  if (update_method_[0] == 'p')
    us = transitions.size();
  else
    for (size_t ii=0; ii < transitions.size(); ++ii)
      if (critique[ii] > 0)
        us++;
        
  if (us > 0)
  {
    representation_->batchWrite(us);
    for (size_t ii=0; ii < transitions.size(); ++ii)
    {
      if (update_method_[0] == 'p' || critique[ii] > 0)
      {
        Vector delta = transitions[ii]->prev_action.v - actions[ii]->v;
        
        if (update_method_[0] == 'p')
          delta = critique[ii] * delta;
          
        if (step_limit_.size())
          for (size_t ii=0; ii < delta.size(); ++ii)
            delta[ii] = fmin(fmax(delta[ii], -step_limit_[ii]), step_limit_[ii]);

        Vector target_u = actions[ii]->v + delta;
        representation_->enqueue(projector_->project(transitions[ii]->prev_obs), target_u);
      }
    }
    representation_->write();
  }
}

void ActionACPredictor::finalize()
{
  Predictor::finalize();
}

void ExpandedActionACPredictor::request(ConfigurationRequest *config)
{
  ActionACPredictor::request(config);

  config->push_back(CRP("discrete_action", "vector", "Discrete action that expands to continuous actor action", discrete_action_));
}

void ExpandedActionACPredictor::configure(Configuration &config)
{
  ActionACPredictor::configure(config);
  
  discrete_action_ = config["discrete_action"].v();
}

void ExpandedActionACPredictor::reconfigure(const Configuration &config)
{
  ActionACPredictor::reconfigure(config);
}

void ExpandedActionACPredictor::update(const Transition &transition)
{
  Predictor::update(transition);
  
  if (step_limit_.size() && step_limit_.size() != transition.obs.u.size())
  {
    if (step_limit_.size() == 1)
      step_limit_ = ConstantVector(transition.obs.u.size(), step_limit_[0]);
    else
      throw bad_param("predictor/ac:step_limit");
  }
    
  ProjectionPtr ap = projector_->project(transition.prev_obs);
  
  Vector u;
  representation_->target()->read(ap, &u);
  
  if (!u.size())
    u = ConstantVector(transition.obs.u.size(), 0.);
  
  double critique = critic_->criticize(transition, discrete_action_);
  
  if (update_method_[0] == 'p' || critique > 0)
  {
    Vector delta = transition.obs.u - u;
    
    if (update_method_[0] == 'p')
      delta = critique * delta;
      
    if (step_limit_.size())
      for (size_t ii=0; ii < delta.size(); ++ii)
        delta[ii] = fmin(fmax(delta[ii], -step_limit_[ii]), step_limit_[ii]);

    Vector target_u = u + delta;
    
    representation_->write(ap, target_u, alpha_);
    representation_->finalize();
  }
}

void ExpandedActionACPredictor::update(const std::vector<const Transition*> &transitions)
{
  if (step_limit_.size() && step_limit_.size() != transitions[0]->obs.u.size())
  {
    if (step_limit_.size() == 1)
      step_limit_ = ConstantVector(transitions[0]->obs.u.size(), step_limit_[0]);
    else
      throw bad_param("predictor/ac:step_limit");
  }
    
  Matrix u;
  representation_->target()->batchRead(transitions.size());
  for (size_t ii=0; ii < transitions.size(); ++ii)
    representation_->target()->enqueue(projector_->project(transitions[ii]->prev_obs));
  representation_->target()->read(&u);
  
  Action a = discrete_action_;
  LargeVector critique = critic_->criticize(transitions, std::vector<const Action*>(transitions.size(), &a));
  
  size_t us=0;
  if (update_method_[0] == 'p')
    us = transitions.size();
  else
    for (size_t ii=0; ii < transitions.size(); ++ii)
      if (critique[ii] > 0)
        us++;
      
  if (us > 0)
  {
    representation_->batchWrite(us);
    for (size_t ii=0; ii < transitions.size(); ++ii)
    {
      if (update_method_[0] == 'p' || critique[ii] > 0)
      {
        Vector delta = transitions[ii]->obs.u - u.row(ii).array();
        
        if (update_method_[0] == 'p')
          delta = critique[ii] * delta;
          
        if (step_limit_.size())
          for (size_t ii=0; ii < delta.size(); ++ii)
            delta[ii] = fmin(fmax(delta[ii], -step_limit_[ii]), step_limit_[ii]);

        Vector target_u = u.row(ii).array() + delta;
        representation_->enqueue(projector_->project(transitions[ii]->prev_obs), target_u);
      }
    }
    representation_->write();
  }
}

void ExpandedActionACPredictor::finalize()
{
  ActionACPredictor::finalize();
}

void ProbabilityACPredictor::request(ConfigurationRequest *config)
{
  Predictor::request(config);
  
  config->push_back(CRP("alpha", "Critic learning rate", alpha_));
  config->push_back(CRP("beta", "Actor learning rate", beta_));
  config->push_back(CRP("gamma", "Discount rate", gamma_));
  config->push_back(CRP("lambda", "Trace decay rate", lambda_));

  config->push_back(CRP("critic_projector", "projector.observation", "Projects observations onto critic representation space", critic_projector_));
  config->push_back(CRP("critic_representation", "representation.value/state", "Value function representation", critic_representation_));
  config->push_back(CRP("critic_trace", "trace", "Trace of critic projections", critic_trace_, true));

  config->push_back(CRP("actor_projector", "projector.pair", "Projects observation-action pairs onto actor representation space", actor_projector_));
  config->push_back(CRP("actor_representation", "representation.value/action", "Action-probability representation", actor_representation_));
  config->push_back(CRP("actor_trace", "trace", "Trace of actor projections", actor_trace_, true));

  config->push_back(CRP("discretizer", "discretizer.action", "Action discretizer", discretizer_));
}

void ProbabilityACPredictor::configure(Configuration &config)
{
  Predictor::configure(config);
  
  critic_projector_ = (Projector*)config["critic_projector"].ptr();
  critic_representation_ = (Representation*)config["critic_representation"].ptr();
  critic_trace_ = (Trace*)config["critic_trace"].ptr();
  
  actor_projector_ = (Projector*)config["actor_projector"].ptr();
  actor_representation_ = (Representation*)config["actor_representation"].ptr();
  actor_trace_ = (Trace*)config["actor_trace"].ptr();

  discretizer_ = (Discretizer*)config["discretizer"].ptr();
  
  alpha_ = config["alpha"];
  beta_ = config["beta"];
  gamma_ = config["gamma"];
  lambda_ = config["lambda"];
}

void ProbabilityACPredictor::reconfigure(const Configuration &config)
{
  Predictor::reconfigure(config);
  
  if (config.has("action") && config["action"].str() == "reset")
    finalize();
}

void ProbabilityACPredictor::update(const Transition &transition)
{
  Predictor::update(transition);

  ProjectionPtr ap = actor_projector_->project(transition.prev_obs, transition.prev_action);
  ProjectionPtr vp = critic_projector_->project(transition.prev_obs);
  
  Vector res;
  double vnext = critic_representation_->read(critic_projector_->project(transition.obs), &res);

  // Calculate target value
  double target = transition.reward;
  if (transition.action.size())
    target += gamma_*vnext;
  double delta = target - critic_representation_->read(vp, &res);
  
  // V update
  critic_representation_->write(vp, VectorConstructor(target), beta_);
  
  // Actor that maps states to a preference value for each action   
  double a_repr = actor_representation_->read(ap, &res);
  
  actor_representation_->write(ap, VectorConstructor(a_repr + alpha_*delta), 1);  
  
  if (critic_trace_)
  {
    critic_representation_->update(*critic_trace_, VectorConstructor(beta_*delta), gamma_*lambda_);
    critic_trace_->add(vp, gamma_*lambda_);
  }
  
  //q_representation_->finalize();
  critic_representation_->finalize();
  actor_representation_->finalize();
  
  //throw Exception("ProbabilityACPredictor::update not implemented");
}

void ProbabilityACPredictor::finalize()
{
  Predictor::finalize();

  if (critic_trace_)
    critic_trace_->clear();
  if (actor_trace_)
    actor_trace_->clear();
}
