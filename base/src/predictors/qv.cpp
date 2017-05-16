/** \file qv.cpp
 * \brief QV predictor source file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-06-30
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

#include <grl/predictors/qv.h>

using namespace grl;

REGISTER_CONFIGURABLE(QVPredictor)
REGISTER_CONFIGURABLE(AVPredictor)

void QVPredictor::request(ConfigurationRequest *config)
{
  CriticPredictor::request(config);

  config->push_back(CRP("alpha", "State-action value learning rate", alpha_));
  config->push_back(CRP("beta", "State value learning rate", beta_));
  config->push_back(CRP("gamma", "Discount rate", gamma_));
  config->push_back(CRP("lambda", "Trace decay rate", lambda_));

  config->push_back(CRP("q_projector", "projector.pair", "Projects observation-action pairs onto representation space", q_projector_));
  config->push_back(CRP("q_representation", "representation.value/action", "State-action value representation (Q)", q_representation_));
  config->push_back(CRP("v_projector", "projector.observation", "Projects observations onto representation space", v_projector_));
  config->push_back(CRP("v_representation", "representation.value/state", "State value representation (V)", v_representation_));
  config->push_back(CRP("trace", "trace", "Trace of projections", trace_, true));
}

void QVPredictor::configure(Configuration &config)
{
  CriticPredictor::configure(config);
  
  q_projector_ = (Projector*)config["q_projector"].ptr();
  q_representation_ = (Representation*)config["q_representation"].ptr();
  v_projector_ = (Projector*)config["v_projector"].ptr();
  v_representation_ = (Representation*)config["v_representation"].ptr();
  trace_ = (Trace*)config["trace"].ptr();
  
  alpha_ = config["alpha"];
  beta_ = config["beta"];
  gamma_ = config["gamma"];
  lambda_ = config["lambda"];
}

void QVPredictor::reconfigure(const Configuration &config)
{
  CriticPredictor::reconfigure(config);
  
  if (config.has("action") && config["action"].str() == "reset")
    finalize();
}

double QVPredictor::criticize(const Transition &transition, const Action &action)
{
  Predictor::update(transition);

  ProjectionPtr qp = q_projector_->project(transition.prev_obs, transition.prev_action);
  ProjectionPtr vp = v_projector_->project(transition.prev_obs);
  
  Vector res;
  double vnext = v_representation_->read(v_projector_->project(transition.obs), &res);

  // Calculate target value
  double target = transition.reward;
  if (transition.action.size())
    target += gamma_*vnext;
  double delta = target - v_representation_->read(vp, &res);

  // Q update  
  q_representation_->write(qp, VectorConstructor(target), alpha_);
  
  // V update
  v_representation_->write(vp, VectorConstructor(target), beta_);
  
  if (trace_)
  {
    v_representation_->update(*trace_, VectorConstructor(beta_*delta), gamma_*lambda_);
    trace_->add(vp, gamma_*lambda_);
  }
  
  q_representation_->finalize();
  v_representation_->finalize();
  
  return delta;
}

void QVPredictor::finalize()
{
  CriticPredictor::finalize();
  
  if (trace_)
    trace_->clear();
}

void AVPredictor::request(ConfigurationRequest *config)
{
  CriticPredictor::request(config);

  config->push_back(CRP("alpha", "State-action advantage learning rate", alpha_));
  config->push_back(CRP("beta", "State value learning rate", beta_));
  config->push_back(CRP("gamma", "Discount rate", gamma_));
  config->push_back(CRP("lambda", "Trace decay rate", lambda_));

  config->push_back(CRP("a_projector", "projector.pair", "Projects observation-action pairs onto representation space", a_projector_));
  config->push_back(CRP("a_representation", "representation.value/action", "State-action advantage representation (A)", a_representation_));
  config->push_back(CRP("v_projector", "projector.observation", "Projects observations onto representation space", v_projector_));
  config->push_back(CRP("v_representation", "representation.value/state", "State value representation (V)", v_representation_));
  config->push_back(CRP("trace", "trace", "Trace of projections", trace_, true));
}

void AVPredictor::configure(Configuration &config)
{
  CriticPredictor::configure(config);
  
  a_projector_ = (Projector*)config["a_projector"].ptr();
  a_representation_ = (Representation*)config["a_representation"].ptr();
  v_projector_ = (Projector*)config["v_projector"].ptr();
  v_representation_ = (Representation*)config["v_representation"].ptr();
  trace_ = (Trace*)config["trace"].ptr();
  
  alpha_ = config["alpha"];
  beta_ = config["beta"];
  gamma_ = config["gamma"];
  lambda_ = config["lambda"];
}

void AVPredictor::reconfigure(const Configuration &config)
{
  CriticPredictor::reconfigure(config);
  
  if (config.has("action") && config["action"].str() == "reset")
    finalize();
}

double AVPredictor::criticize(const Transition &transition, const Action &action)
{
  Predictor::update(transition);

  ProjectionPtr ap = a_projector_->project(transition.prev_obs, transition.prev_action);
  ProjectionPtr vp = v_projector_->project(transition.prev_obs);
  
  Vector res;
  double vnext = v_representation_->read(v_projector_->project(transition.obs), &res);

  // Calculate target value
  double target = transition.reward;
  if (transition.action.size())
    target += gamma_*vnext;
  double delta = target - v_representation_->read(vp, &res);

  // A update  
  a_representation_->write(ap, VectorConstructor(delta), alpha_);

  // V update
  v_representation_->write(vp, VectorConstructor(target), beta_);
  
  if (trace_)
  {
    v_representation_->update(*trace_, VectorConstructor(beta_*delta), gamma_*lambda_);
    trace_->add(vp, gamma_*lambda_);
  }
  
  a_representation_->finalize();
  v_representation_->finalize();
  
  return delta;
}

void AVPredictor::finalize()
{
  CriticPredictor::finalize();
  
  if (trace_)
    trace_->clear();
}
