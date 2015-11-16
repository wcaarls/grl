/** \file sarsa.cpp
 * \brief SARSA predictor source file.
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

#include <grl/predictors/sarsa.h>

using namespace grl;

REGISTER_CONFIGURABLE(SARSAPredictor)
REGISTER_CONFIGURABLE(ExpectedSARSAPredictor)
REGISTER_CONFIGURABLE(UncoupledSARSAPredictor)

void SARSAPredictor::request(ConfigurationRequest *config)
{
  Predictor::request(config);
  
  config->push_back(CRP("alpha", "Learning rate", alpha_));
  config->push_back(CRP("gamma", "Discount rate", gamma_));
  config->push_back(CRP("lambda", "Trace decay rate", lambda_));

  config->push_back(CRP("projector", "projector.pair", "Projects observation-action pairs onto representation space", projector_));
  config->push_back(CRP("representation", "representation.value/action", "Q-value representation", representation_));
  config->push_back(CRP("trace", "trace", "Trace of projections", trace_));
}

void SARSAPredictor::configure(Configuration &config)
{
  Predictor::configure(config);
  
  projector_ = (Projector*)config["projector"].ptr();
  representation_ = (Representation*)config["representation"].ptr();
  trace_ = (Trace*)config["trace"].ptr();
  
  alpha_ = config["alpha"];
  gamma_ = config["gamma"];
  lambda_ = config["lambda"];
}

void SARSAPredictor::reconfigure(const Configuration &config)
{
  Predictor::reconfigure(config);
  
  if (config.has("action") && config["action"].str() == "reset")
    finalize();
}

SARSAPredictor *SARSAPredictor::clone() const
{
  SARSAPredictor *sp = new SARSAPredictor();
  sp->projector_ = projector_->clone();
  sp->representation_= representation_->clone();
  sp->trace_ = trace_->clone();
  return sp;
}

void SARSAPredictor::update(const Transition &transition)
{
  Predictor::update(transition);

  ProjectionPtr p = projector_->project(transition.prev_obs, transition.prev_action);
  Vector q;

  double target = transition.reward;
  if (transition.obs.size())
    target += gamma_*representation_->read(projector_->project(transition.obs, transition.action), &q);
  double delta = target - representation_->read(p, &q);
  
  representation_->write(p, VectorConstructor(target), alpha_);

  // TODO: recently added point is not in trace
  representation_->update(*trace_, VectorConstructor(alpha_*delta), gamma_*lambda_);
  trace_->add(p, gamma_*lambda_);
  
  representation_->finalize();
}

void SARSAPredictor::finalize()
{
  Predictor::finalize();
  
  trace_->clear();
}

void ExpectedSARSAPredictor::request(ConfigurationRequest *config)
{
  Predictor::request(config);
  
  config->push_back(CRP("alpha", "Learning rate", alpha_));
  config->push_back(CRP("gamma", "Discount rate", gamma_));
  config->push_back(CRP("lambda", "Trace decay rate", lambda_));

  config->push_back(CRP("projector", "projector.pair", "Projects observation-action pairs onto representation space", projector_));
  config->push_back(CRP("representation", "representation.value/action", "Q-value representation", representation_));
  config->push_back(CRP("policy", "policy/discrete/q", "Q-value based policy", policy_));
  config->push_back(CRP("sampler", "sampler", "Target distribution", sampler_));
  config->push_back(CRP("trace", "trace", "Trace of projections", trace_));
}

void ExpectedSARSAPredictor::configure(Configuration &config)
{
  Predictor::configure(config);
  
  projector_ = (Projector*)config["projector"].ptr();
  representation_ = (Representation*)config["representation"].ptr();
  policy_ = (QPolicy*)config["policy"].ptr();
  sampler_ = (Sampler*)config["sampler"].ptr();
  trace_ = (Trace*)config["trace"].ptr();
  
  alpha_ = config["alpha"];
  gamma_ = config["gamma"];
  lambda_ = config["lambda"];
}

void ExpectedSARSAPredictor::reconfigure(const Configuration &config)
{
  Predictor::reconfigure(config);
  
}

ExpectedSARSAPredictor *ExpectedSARSAPredictor::clone() const
{
  ExpectedSARSAPredictor *sp = new ExpectedSARSAPredictor();
  sp->projector_ = projector_->clone();
  sp->representation_= representation_->clone();
  sp->policy_ = policy_->clone();
  sp->sampler_ = sampler_->clone();
  sp->trace_ = trace_->clone();
  return sp;
}

void ExpectedSARSAPredictor::update(const Transition &transition)
{
  Predictor::update(transition);

  ProjectionPtr p = projector_->project(transition.prev_obs, transition.prev_action);

  double target = transition.reward;
  
  if (transition.obs.size())
  {
    Vector values, distribution;
    policy_->values(transition.obs, &values);
    sampler_->distribution(values, &distribution);

    double v = 0.;
    for (size_t ii=0; ii < values.size(); ++ii)
      v += values[ii]*distribution[ii];

    target += gamma_*v;
  }
  
  Vector q;
  double delta = target - representation_->read(p, &q);

  representation_->write(p, VectorConstructor(target), alpha_);
  representation_->update(*trace_, VectorConstructor(alpha_*delta), gamma_*lambda_);
  trace_->add(p, gamma_*lambda_);
  
  representation_->finalize();
}

void ExpectedSARSAPredictor::finalize()
{
  Predictor::finalize();
  
  trace_->clear();
}

void UncoupledSARSAPredictor::request(ConfigurationRequest *config)
{
  SARSAPredictor::request(config);
  
  config->push_back(CRP("update_representation", "representation.value/action", "Q-value representation used to calculate the update", update_representation_));
}

void UncoupledSARSAPredictor::configure(Configuration &config)
{
  SARSAPredictor::configure(config);
  
  update_representation_ = (Representation*)config["update_representation"].ptr();
}

void UncoupledSARSAPredictor::reconfigure(const Configuration &config)
{
  SARSAPredictor::reconfigure(config);
}

UncoupledSARSAPredictor *UncoupledSARSAPredictor::clone() const
{
  return NULL;
}

void UncoupledSARSAPredictor::update(const Transition &transition)
{
  Predictor::update(transition);

  ProjectionPtr p = projector_->project(transition.prev_obs, transition.prev_action);
  Vector q;

  double target = transition.reward;
  if (transition.obs.size())
    target += gamma_*update_representation_->read(projector_->project(transition.obs, transition.action), &q);
  double delta = target - update_representation_->read(p, &q);

  // Should be just update that adds point to LLR memory  
  representation_->read(p, &q);
  representation_->write(p, VectorConstructor(q[0]+delta), alpha_);

  // TODO: recently added point is not in trace
  representation_->update(*trace_, VectorConstructor(alpha_*delta), gamma_*lambda_);
  trace_->add(p, gamma_*lambda_);
  
  representation_->finalize();
}

void UncoupledSARSAPredictor::finalize()
{
  SARSAPredictor::finalize();
}
