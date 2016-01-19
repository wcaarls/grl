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

void QVPredictor::request(ConfigurationRequest *config)
{
  Predictor::request(config);

  config->push_back(CRP("alpha", "State-action value learning rate", alpha_));
  config->push_back(CRP("beta", "State value learning rate", beta_));
  config->push_back(CRP("gamma", "Discount rate", gamma_));
  config->push_back(CRP("lambda", "Trace decay rate", lambda_));

  config->push_back(CRP("q_projector", "projector.pair", "Projects observation-action pairs onto representation space", q_projector_));
  config->push_back(CRP("q_representation", "representation.value/action", "State-action value representation (Q)", q_representation_));
  config->push_back(CRP("v_projector", "projector.observation", "Projects observations onto representation space", v_projector_));
  config->push_back(CRP("v_representation", "representation.value/state", "State value representation (V)", v_representation_));
  config->push_back(CRP("trace", "trace", "Trace of projections", trace_));
}

void QVPredictor::configure(Configuration &config)
{
  Predictor::configure(config);
  
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
  Predictor::reconfigure(config);
  
  if (config.has("action") && config["action"].str() == "reset")
    finalize();
}

QVPredictor *QVPredictor::clone() const
{
  return NULL;
}

void QVPredictor::update(const Transition &transition)
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

  // Q update  
  q_representation_->write(qp, VectorConstructor(target), alpha_);
  
  // V update
  v_representation_->write(vp, VectorConstructor(target), beta_);
  
  double delta = target - v_representation_->read(vp, &res);
  v_representation_->update(*trace_, VectorConstructor(beta_*delta), gamma_*lambda_);
  trace_->add(vp, gamma_*lambda_);
  
  q_representation_->finalize();
  v_representation_->finalize();
}

void QVPredictor::finalize()
{
  Predictor::finalize();
  
  trace_->clear();
}
