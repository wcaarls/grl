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

void SARSAPredictor::request(ConfigurationRequest *config)
{
  CriticPredictor::request(config);
  
  config->push_back(CRP("alpha", "Learning rate", alpha_));
  config->push_back(CRP("gamma", "Discount rate", gamma_));
  config->push_back(CRP("lambda", "Trace decay rate", lambda_));

  config->push_back(CRP("projector", "projector.pair", "Projects observation-action pairs onto representation space", projector_));
  config->push_back(CRP("representation", "representation.value/action", "Q-value representation", representation_));
  config->push_back(CRP("trace", "trace", "Trace of projections", trace_, true));
}

void SARSAPredictor::configure(Configuration &config)
{
  CriticPredictor::configure(config);
  
  projector_ = (Projector*)config["projector"].ptr();
  representation_ = (Representation*)config["representation"].ptr();
  trace_ = (Trace*)config["trace"].ptr();
  
  alpha_ = config["alpha"];
  gamma_ = config["gamma"];
  lambda_ = config["lambda"];
}

void SARSAPredictor::reconfigure(const Configuration &config)
{
  CriticPredictor::reconfigure(config);
  
  if (config.has("action") && config["action"].str() == "reset")
    finalize();
}

void SARSAPredictor::update(const std::vector<const Transition*> &transitions)
{
  Matrix q;
  
  size_t qs = 0;
  for (size_t ii=0; ii < transitions.size(); ++ii)
    if (!transitions[ii]->obs.absorbing)
      qs++;
    
  representation_->batchRead(qs);
  for (size_t ii=0; ii < transitions.size(); ++ii)
    if (!transitions[ii]->obs.absorbing)
      representation_->enqueue(projector_->project(transitions[ii]->obs, transitions[ii]->action));
  representation_->read(&q);
  
  qs = 0;
  representation_->batchWrite(transitions.size());
  for (size_t ii=0; ii < transitions.size(); ++ii)
  {
    double target = transitions[ii]->reward;
  
    if (!transitions[ii]->obs.absorbing)
      target += gamma_*q(qs++, 0);
      
    representation_->enqueue(projector_->project(transitions[ii]->prev_obs, transitions[ii]->prev_action), VectorConstructor(target));
  }
  representation_->write();
}

double SARSAPredictor::criticize(const Transition &transition, const Action &action)
{
  Predictor::update(transition);

  ProjectionPtr p = projector_->project(transition.prev_obs, transition.prev_action);
  Vector q;

  double target = transition.reward;
  if (transition.action.size())
    target += gamma_*representation_->read(projector_->project(transition.obs, transition.action), &q);
  double delta = target - representation_->read(p, &q);
  
  representation_->write(p, VectorConstructor(target), alpha_);

  if (trace_)
  {
    representation_->update(*trace_, VectorConstructor(alpha_*delta), gamma_*lambda_);
    trace_->add(p, gamma_*lambda_);
  }
  
  representation_->finalize();

  if (action.size())  
    return target - representation_->read(projector_->project(transition.prev_obs, action), &q);
  else
    return 0;
}

void SARSAPredictor::finalize()
{
  CriticPredictor::finalize();
  
  if (trace_)
    trace_->clear();
}

void ExpectedSARSAPredictor::request(ConfigurationRequest *config)
{
  CriticPredictor::request(config);
  
  config->push_back(CRP("alpha", "Learning rate", alpha_));
  config->push_back(CRP("gamma", "Discount rate", gamma_));
  config->push_back(CRP("lambda", "Trace decay rate", lambda_));

  config->push_back(CRP("projector", "projector.pair", "Projects observation-action pairs onto representation space", projector_));
  config->push_back(CRP("representation", "representation.value/action", "Q-value representation", representation_));
  config->push_back(CRP("policy", "mapping/policy/value", "Value based target policy", policy_));
  config->push_back(CRP("trace", "trace", "Trace of projections", trace_, true));
}

void ExpectedSARSAPredictor::configure(Configuration &config)
{
  CriticPredictor::configure(config);
  
  projector_ = (Projector*)config["projector"].ptr();
  representation_ = (Representation*)config["representation"].ptr();
  policy_ = (ValuePolicy*)config["policy"].ptr();
  trace_ = (Trace*)config["trace"].ptr();
  
  alpha_ = config["alpha"];
  gamma_ = config["gamma"];
  lambda_ = config["lambda"];
}

void ExpectedSARSAPredictor::reconfigure(const Configuration &config)
{
  CriticPredictor::reconfigure(config);
}

double ExpectedSARSAPredictor::criticize(const Transition &transition, const Action &action)
{
  Predictor::update(transition);

  ProjectionPtr p = projector_->project(transition.prev_obs, transition.prev_action);

  double target = transition.reward;
  
  if (transition.action.size())
    target += gamma_*policy_->value(transition.obs);  
    
  Vector q;
  double delta = target - representation_->read(p, &q);

  representation_->write(p, VectorConstructor(target), alpha_);
  if (trace_)
  {
    representation_->update(*trace_, VectorConstructor(alpha_*delta), gamma_*lambda_);
    trace_->add(p, gamma_*lambda_);
  }
  
  representation_->finalize();
  
  if (action.size())  
    return target - representation_->read(projector_->project(transition.prev_obs, action), &q);
  else
    return 0;
}

void ExpectedSARSAPredictor::finalize()
{
  CriticPredictor::finalize();
  
  if (trace_)
    trace_->clear();
}
