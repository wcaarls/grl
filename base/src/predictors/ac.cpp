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
REGISTER_CONFIGURABLE(ProbabilityACPredictor)
REGISTER_CONFIGURABLE(QACPredictor)
REGISTER_CONFIGURABLE(QVACPredictor)

void ActionACPredictor::request(ConfigurationRequest *config)
{
  Predictor::request(config);

  config->push_back(CRP("alpha", "Critic learning rate", alpha_));
  config->push_back(CRP("beta", "Actor learning rate", beta_));
  config->push_back(CRP("gamma", "Discount rate", gamma_));
  config->push_back(CRP("lambda", "Trace decay rate", lambda_));

  config->push_back(CRP("update_method", "Actor update method", update_method_, CRP::Configuration, {"proportional", "cacla"}));
  config->push_back(CRP("step_limit", "Actor exploration step limit", step_limit_));

  config->push_back(CRP("critic_projector", "projector.observation", "Projects observations onto critic representation space", critic_projector_));
  config->push_back(CRP("critic_representation", "representation.value/state", "Value function representation", critic_representation_));
  config->push_back(CRP("critic_trace", "trace", "Trace of critic projections", critic_trace_, true));

  config->push_back(CRP("actor_projector", "projector.observation", "Projects observations onto actor representation space", actor_projector_));
  config->push_back(CRP("actor_representation", "representation.action", "Action representation", actor_representation_));
  config->push_back(CRP("actor_trace", "trace", "Trace of actor projections", actor_trace_, true));
}

void ActionACPredictor::configure(Configuration &config)
{
  Predictor::configure(config);

  critic_projector_ = (Projector*)config["critic_projector"].ptr();
  critic_representation_ = (Representation*)config["critic_representation"].ptr();
  critic_trace_ = (Trace*)config["critic_trace"].ptr();
  
  actor_projector_ = (Projector*)config["actor_projector"].ptr();
  actor_representation_ = (Representation*)config["actor_representation"].ptr();
  actor_trace_ = (Trace*)config["actor_trace"].ptr();
  
  alpha_ = config["alpha"];
  beta_ = config["beta"];
  gamma_ = config["gamma"];
  lambda_ = config["lambda"];
  
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

  // (LLR)   obtain buckets with nearest neighbours
  // (SARSA) obtain tile indices that point to previous observations
  ProjectionPtr cp = critic_projector_->project(transition.prev_obs);
  ProjectionPtr ap = actor_projector_->project(transition.prev_obs);
  Vector v, u, target_u;
  
  double target = transition.reward;
  if (transition.action.size())
    target += gamma_*critic_representation_->read(critic_projector_->project(transition.obs), &v);
  double delta = target - critic_representation_->read(cp, &v);
  // Add LLR sample to DB of samples
  critic_representation_->write(cp, VectorConstructor(target), alpha_);
  if (critic_trace_)
  {
    critic_representation_->update(*critic_trace_, VectorConstructor(alpha_*delta), gamma_*lambda_);
    critic_trace_->add(cp, gamma_*lambda_);
  }

  if (update_method_[0] == 'p' || delta > 0)
  {
    actor_representation_->read(ap, &u);
    if (!u.size())
      u = ConstantVector(transition.prev_action.size(), 0.);
    Vector Delta = (transition.prev_action.v - u);
    
    if (step_limit_.size())
      for (size_t ii=0; ii < Delta.size(); ++ii)
        Delta[ii] = fmin(fmax(Delta[ii], -step_limit_[ii]), step_limit_[ii]);

    if (update_method_[0] == 'p')
      Delta = delta * Delta;

    target_u = u + Delta;

    actor_representation_->write(ap, target_u, beta_);
    if (actor_trace_)
    {
      actor_representation_->update(*actor_trace_, beta_*(target_u-u), gamma_*lambda_);
      actor_trace_->add(ap, gamma_*lambda_);
    }
    
    actor_representation_->finalize();
  }
  
  critic_representation_->finalize();
}

void ActionACPredictor::finalize()
{
  Predictor::finalize();

  if (critic_trace_)
    critic_trace_->clear();
  if (actor_trace_)
    actor_trace_->clear();
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

  throw Exception("ProbabilityACPredictor::update not implemented");
}

void ProbabilityACPredictor::finalize()
{
  Predictor::finalize();

  if (critic_trace_)
    critic_trace_->clear();
  if (actor_trace_)
    actor_trace_->clear();
}

void QACPredictor::request(ConfigurationRequest *config)
{
  Predictor::request(config);

  config->push_back(CRP("alpha", "Critic learning rate", alpha_));
  config->push_back(CRP("beta", "Actor learning rate", beta_));
  config->push_back(CRP("gamma", "Discount rate", gamma_));
  config->push_back(CRP("lambda", "Trace decay rate", lambda_));
  config->push_back(CRP("kappa", "Advantage scaling factor", kappa_));

  config->push_back(CRP("update_method", "Actor update method", update_method_, CRP::Configuration, {"proportional", "cacla"}));
  config->push_back(CRP("step_limit", "Actor exploration step limit", step_limit_));

  config->push_back(CRP("target", "mapping", "Target value at next state", target_));

  config->push_back(CRP("critic_projector", "projector.pair", "Projects observations onto critic representation space", critic_projector_));
  config->push_back(CRP("critic_representation", "representation.value/action", "Value function representation", critic_representation_));
  config->push_back(CRP("critic_trace", "trace", "Trace of critic projections", critic_trace_, true));

  config->push_back(CRP("actor_projector", "projector.observation", "Projects observations onto actor representation space", actor_projector_));
  config->push_back(CRP("actor_representation", "representation.action", "Action representation", actor_representation_));
  config->push_back(CRP("actor_trace", "trace", "Trace of actor projections", actor_trace_, true));
}

void QACPredictor::configure(Configuration &config)
{
  Predictor::configure(config);
  
  target_ = (Mapping*)config["target"].ptr();

  critic_projector_ = (Projector*)config["critic_projector"].ptr();
  critic_representation_ = (Representation*)config["critic_representation"].ptr();
  critic_trace_ = (Trace*)config["critic_trace"].ptr();
  
  actor_projector_ = (Projector*)config["actor_projector"].ptr();
  actor_representation_ = (Representation*)config["actor_representation"].ptr();
  actor_trace_ = (Trace*)config["actor_trace"].ptr();
  
  alpha_ = config["alpha"];
  beta_ = config["beta"];
  gamma_ = config["gamma"];
  lambda_ = config["lambda"];
  kappa_ = config["kappa"];
  
  update_method_ = config["update_method"].str();
  step_limit_ = config["step_limit"].v();
}

void QACPredictor::reconfigure(const Configuration &config)
{
  Predictor::reconfigure(config);

  if (config.has("action") && config["action"].str() == "reset")
    finalize();
}

void QACPredictor::update(const Transition &transition)
{
  Predictor::update(transition);
  
  if (step_limit_.size() && step_limit_.size() != transition.prev_action.size())
  {
    if (step_limit_.size() == 1)
      step_limit_ = ConstantVector(transition.prev_action.size(), step_limit_[0]);
    else
      throw bad_param("predictor/ac/q:step_limit");
  }
  
  Vector v, u, delta_u, target_u;
  ProjectionPtr ap = actor_projector_->project(transition.prev_obs);
  actor_representation_->read(ap, &u);
  if (!u.size())
    u = ConstantVector(transition.prev_action.size(), 0.);
  ProjectionPtr cp  = critic_projector_->project(transition.prev_obs, transition.prev_action),
                cpu = critic_projector_->project(transition.prev_obs, u);
                
  double prev_v = target_->read(transition.prev_obs, &v);
  double target = prev_v + (transition.reward - prev_v)/kappa_;
  
  if (transition.action.size())
    target += gamma_*target_->read(transition.obs, &v);
  
  // TD error between target and executed action
  double delta  = target - critic_representation_->read(cp, &v);
  
  // TD error between target and desired action
  double deltau = target - critic_representation_->read(cpu, &v);
  
  // Update critic based on executed action TD error
  critic_representation_->write(cp, VectorConstructor(target), alpha_);
  if (critic_trace_)
  {
    critic_representation_->update(*critic_trace_, VectorConstructor(alpha_*delta), gamma_*lambda_);
    critic_trace_->add(cp, gamma_*lambda_);
  }
    
  // Update actor based on desired action TD error
  if (update_method_[0] == 'p' || deltau > 0)
  {
    Vector Delta = (transition.prev_action.v - u);
    
    if (step_limit_.size())
      for (size_t ii=0; ii < Delta.size(); ++ii)
        Delta[ii] = fmin(fmax(Delta[ii], -step_limit_[ii]), step_limit_[ii]);

    if (update_method_[0] == 'p')
      Delta = deltau * Delta;

    target_u = u + Delta;

    actor_representation_->write(ap, target_u, beta_);
    if (actor_trace_)
    {
      actor_representation_->update(*actor_trace_, beta_*(target_u-u), gamma_*lambda_);
      actor_trace_->add(ap, gamma_*lambda_);
    }
    
    actor_representation_->finalize();
  }
  
  critic_representation_->finalize();
}

void QACPredictor::finalize()
{
  Predictor::finalize();

  if (critic_trace_)
    critic_trace_->clear();
  if (actor_trace_)
    actor_trace_->clear();
}

// QVAC

void QVACPredictor::request(ConfigurationRequest *config)
{
  Predictor::request(config);

  config->push_back(CRP("alpha", "Critic Q learning rate", alpha_));
  config->push_back(CRP("beta_v", "Critic V learning rate", beta_v_));
  config->push_back(CRP("beta_a", "Actor learning rate", beta_a_));
  config->push_back(CRP("gamma", "Discount rate", gamma_));
  config->push_back(CRP("lambda", "Trace decay rate", lambda_));

  config->push_back(CRP("update_method", "Actor update method", update_method_, CRP::Configuration, {"proportional", "cacla"}));
  config->push_back(CRP("step_limit", "Actor exploration step limit", step_limit_));

  config->push_back(CRP("critic_q_projector", "projector.pair", "Projects observations onto critic Q representation space", critic_q_projector_));
  config->push_back(CRP("critic_q_representation", "representation.value/action", "Q Value function representation", critic_q_representation_));

  config->push_back(CRP("critic_v_projector", "projector.observation", "Projects observations onto critic V representation space", critic_v_projector_));
  config->push_back(CRP("critic_v_representation", "representation.value/state", "V Value function representation", critic_v_representation_));
  config->push_back(CRP("critic_v_trace", "trace", "Trace of critic V projections", critic_v_trace_, true));

  config->push_back(CRP("actor_projector", "projector.observation", "Projects observations onto actor representation space", actor_projector_));
  config->push_back(CRP("actor_representation", "representation.action", "Action representation", actor_representation_));
  config->push_back(CRP("actor_trace", "trace", "Trace of actor projections", actor_trace_, true));
}

void QVACPredictor::configure(Configuration &config)
{
  Predictor::configure(config);
  
  critic_q_projector_ = (Projector*)config["critic_q_projector"].ptr();
  critic_q_representation_ = (Representation*)config["critic_q_representation"].ptr();
  
  critic_v_projector_ = (Projector*)config["critic_v_projector"].ptr();
  critic_v_representation_ = (Representation*)config["critic_v_representation"].ptr();
  critic_v_trace_ = (Trace*)config["critic_v_trace"].ptr();
  
  actor_projector_ = (Projector*)config["actor_projector"].ptr();
  actor_representation_ = (Representation*)config["actor_representation"].ptr();
  actor_trace_ = (Trace*)config["actor_trace"].ptr();
  
  alpha_ = config["alpha"];
  beta_v_ = config["beta_v"];
  beta_a_ = config["beta_a"];
  gamma_ = config["gamma"];
  lambda_ = config["lambda"];
  
  update_method_ = config["update_method"].str();
  step_limit_ = config["step_limit"].v();
}

void QVACPredictor::reconfigure(const Configuration &config)
{
  Predictor::reconfigure(config);

  if (config.has("action") && config["action"].str() == "reset")
    finalize();
}

void QVACPredictor::update(const Transition &transition)
{
  Predictor::update(transition);
  
  if (step_limit_.size() && step_limit_.size() != transition.prev_action.size())
  {
    if (step_limit_.size() == 1)
      step_limit_ = ConstantVector(transition.prev_action.size(), step_limit_[0]);
    else
      throw bad_param("predictor/ac/qv:step_limit");
  }

  Vector v, u, delta_u, target_u;
  ProjectionPtr ap = actor_projector_->project(transition.prev_obs);
  actor_representation_->read(ap, &u);
  if (!u.size())
    u = ConstantVector(transition.prev_action.size(), 0.);
  ProjectionPtr cp  = critic_q_projector_->project(transition.prev_obs, transition.prev_action),
                cpu = critic_q_projector_->project(transition.prev_obs, u),
                cpv = critic_v_projector_->project(transition.prev_obs);
                
  double target = transition.reward;
  if (transition.action.size())
    target += gamma_*critic_v_representation_->read(critic_v_projector_->project(transition.obs), &v);  
    
  // TD error, or: advantage of taken action over current policy
  double delta = target - critic_v_representation_->read(cpv, &v);
  
  // Advantage of taken action over desired action
  double deltau = delta - critic_q_representation_->read(cpu, &v);
  
  // Update critic based on taken action TD error
  critic_q_representation_->write(cp,  VectorConstructor(delta), alpha_);
  critic_v_representation_->write(cpv, VectorConstructor(target), beta_v_);
  
  if (critic_v_trace_)
  {
    critic_v_representation_->update(*critic_v_trace_, VectorConstructor(beta_v_*delta), gamma_*lambda_);
    critic_v_trace_->add(cpv, gamma_*lambda_);
  }
  
  // Update actor based on desired action TD error
  if (update_method_[0] == 'p' || deltau > 0)
  {
    Vector Delta = (transition.prev_action.v - u);
    
    if (step_limit_.size())
      for (size_t ii=0; ii < Delta.size(); ++ii)
        Delta[ii] = fmin(fmax(Delta[ii], -step_limit_[ii]), step_limit_[ii]);

    if (update_method_[0] == 'p')
      Delta = deltau * Delta;

    target_u = u + Delta;

    actor_representation_->write(ap, target_u, beta_a_);
    if (actor_trace_)
    {
      actor_representation_->update(*actor_trace_, beta_a_*(target_u-u), gamma_*lambda_);
      actor_trace_->add(ap, gamma_*lambda_);
    }
    
    actor_representation_->finalize();
  }
  
  critic_q_representation_->finalize();
  critic_v_representation_->finalize();
}

void QVACPredictor::finalize()
{
  Predictor::finalize();

  if (critic_v_trace_)
    critic_v_trace_->clear();
  if (actor_trace_)
    actor_trace_->clear();
}
