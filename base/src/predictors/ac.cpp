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

void ActionACPredictor::request(ConfigurationRequest *config)
{
  Predictor::request(config);

  config->push_back(CRP("alpha", "Critic learning rate", alpha_));
  config->push_back(CRP("beta", "Actor learning rate", beta_));
  config->push_back(CRP("gamma", "Discount rate", gamma_));
  config->push_back(CRP("lambda", "Trace decay rate", lambda_));

  config->push_back(CRP("critic_projector", "projector.observation", "Projects observations onto critic representation space", critic_projector_));
  config->push_back(CRP("critic_representation", "representation.value/state", "Value function representation", critic_representation_));
  config->push_back(CRP("critic_trace", "trace", "Trace of critic projections", critic_trace_));

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
}

void ActionACPredictor::reconfigure(const Configuration &config)
{
  Predictor::reconfigure(config);

  if (config.has("action") && config["action"].str() == "reset")
    finalize();
}

ActionACPredictor *ActionACPredictor::clone() const
{
  ActionACPredictor *aap = new ActionACPredictor(*this);
  aap->critic_projector_ = critic_projector_->clone();
  aap->critic_representation_= critic_representation_->clone();
  aap->critic_trace_ = critic_trace_->clone();
  aap->actor_projector_ = actor_projector_->clone();
  aap->actor_representation_= actor_representation_->clone();
  if (actor_trace_)
    aap->actor_trace_ = actor_trace_->clone();
  return aap;
}

void ActionACPredictor::update(const Transition &transition)
{
  Predictor::update(transition);

  // (LLR) obtain buckets with nearest neighbours
  ProjectionPtr cp = critic_projector_->project(transition.prev_obs);
  ProjectionPtr ap = actor_projector_->project(transition.prev_obs);
  Vector v, u, delta_u, target_u;
  
  double target = transition.reward;
  if (transition.action.size())
    target += gamma_*critic_representation_->read(critic_projector_->project(transition.obs), &v);
  double delta = target - critic_representation_->read(cp, &v);
  
  // Add LLR sample to DB of samples
  critic_representation_->write(cp, VectorConstructor(target), alpha_);
  // ???
  critic_representation_->update(*critic_trace_, VectorConstructor(alpha_*delta), gamma_*lambda_);
  //
  critic_trace_->add(cp, gamma_*lambda_);
  
  actor_representation_->read(ap, &u);
  if (!u.size())
    u = ConstantVector(transition.prev_action.size(), 0.);
  target_u = u + delta*(transition.prev_action - u);

  actor_representation_->write(ap, target_u, beta_);
  if (actor_trace_)
  {
    actor_representation_->update(*actor_trace_, beta_*(target_u-u), gamma_*lambda_);
    actor_trace_->add(ap, gamma_*lambda_);
  }
  
  critic_representation_->finalize();
  actor_representation_->finalize();
}

void ActionACPredictor::finalize()
{
  Predictor::finalize();

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
  config->push_back(CRP("critic_trace", "trace", "Trace of critic projections", critic_trace_));

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

ProbabilityACPredictor *ProbabilityACPredictor::clone() const
{
  ProbabilityACPredictor *pap = new ProbabilityACPredictor(*this);
  pap->critic_projector_ = critic_projector_->clone();
  pap->critic_representation_= critic_representation_->clone();
  pap->critic_trace_ = critic_trace_->clone();
  pap->actor_projector_ = actor_projector_->clone();
  pap->actor_representation_= actor_representation_->clone();
  if (actor_trace_)
    pap->actor_trace_ = actor_trace_->clone();
  return pap;
}

void ProbabilityACPredictor::update(const Transition &transition)
{
  Predictor::update(transition);

  throw Exception("ProbabilityACPredictor::update not implemented");
}

void ProbabilityACPredictor::finalize()
{
  Predictor::finalize();

  critic_trace_->clear();
  if (actor_trace_)
    actor_trace_->clear();
}
