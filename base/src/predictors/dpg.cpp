/** \file dpg.cpp
 * \brief Deterministic Policy Gradient predictor source file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2016-12-26
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

#include <grl/predictors/dpg.h>

using namespace grl;

REGISTER_CONFIGURABLE(DPGPredictor)

void DPGPredictor::request(ConfigurationRequest *config)
{
  Predictor::request(config);

  config->push_back(CRP("alpha", "Advantage model learning rate", alpha_));
  config->push_back(CRP("beta_v", "Critic learning rate", beta_v_));
  config->push_back(CRP("beta_a", "Actor learning rate", beta_a_));
  config->push_back(CRP("gamma", "Discount rate", gamma_));
  config->push_back(CRP("lambda", "Trace decay rate", lambda_));
  
  config->push_back(CRP("target", "Policy target", target_, CRP::Configuration, {"on-policy", "off-policy"}));

  config->push_back(CRP("projector", "projector.observation", "Projects observations onto representation spaces", projector_));
  config->push_back(CRP("critic_representation", "representation.value/state", "State value function representation", critic_representation_));
  config->push_back(CRP("critic_trace", "trace", "Trace of critic projections", critic_trace_, true));
  config->push_back(CRP("advantage_representation", "representation", "Local advantage model representation (one output per action dimension)", advantage_representation_));
  config->push_back(CRP("actor_representation", "representation.action", "Action representation", actor_representation_));
}

void DPGPredictor::configure(Configuration &config)
{
  Predictor::configure(config);
  
  projector_ = (Projector*)config["projector"].ptr();
  critic_representation_ = (Representation*)config["critic_representation"].ptr();
  critic_trace_ = (Trace*)config["critic_trace"].ptr();
  advantage_representation_ = (Representation*)config["advantage_representation"].ptr();
  actor_representation_ = (Representation*)config["actor_representation"].ptr();
  
  alpha_ = config["alpha"];
  beta_v_ = config["beta_v"];
  beta_a_ = config["beta_a"];
  gamma_ = config["gamma"];
  lambda_ = config["lambda"];
  target_ = config["target"].str();
  
  if (target_ == "on-policy")
    policy_target_ = OnPolicyTarget;
  else
    policy_target_ = OffPolicyTarget;
}

void DPGPredictor::reconfigure(const Configuration &config)
{
  Predictor::reconfigure(config);

  if (config.has("action") && config["action"].str() == "reset")
    finalize();
}

void DPGPredictor::update(const Transition &transition)
{
  Predictor::update(transition);

  ProjectionPtr p = projector_->project(transition.prev_obs);
  Vector mu, q, _;
  
  actor_representation_->read(p, &mu);
  advantage_representation_->read(p, &q);
  double v = critic_representation_->read(p, &_);
  
  // amu is the coordinate for the linear local advantage model q.
  Vector amu = transition.prev_action.v - mu;
  
  // Here we "read out" the local advantage model by taking the dot product.
  double delta = transition.reward - (dot(q, amu) + v);
  
  if (transition.action.size())
  {
    ProjectionPtr pp = projector_->project(transition.obs);
    double vp = critic_representation_->target()->read(pp, &_);
    delta += gamma_ * vp;
  
    if (policy_target_ == OnPolicyTarget)
    {
      Vector mup, qp;
    
      actor_representation_->target()->read(pp, &mup);
      advantage_representation_->target()->read(pp, &qp);
    
      // For on-policy critic update, adjust for advantage of taken
      // action over policy action
      Vector amup = transition.action.v - mup;
      delta += gamma_*dot(qp, amup);
    }
  }
  
  // The actor moves towards the advantage gradient w.r.t the action
  // which, since the model is linear, are just the model parameters.
  actor_representation_->update(p, beta_a_*q);
  
  // The advantage model moves towards the advantage gradient w.r.t.
  // the model parameters, weighted by the temporal difference error.
  advantage_representation_->update(p, alpha_*delta*amu);
  
  // The critic just moves towards the temporal difference error.
  if (critic_trace_)
  {
    critic_trace_->add(p, gamma_*lambda_);
    critic_representation_->update(*critic_trace_, VectorConstructor(beta_v_*delta));
  }
  else
    critic_representation_->update(p, VectorConstructor(beta_v_*delta));
}

void DPGPredictor::finalize()
{
  Predictor::finalize();

  if (critic_trace_)
    critic_trace_->clear();
}
