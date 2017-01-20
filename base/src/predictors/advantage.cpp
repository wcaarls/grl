/** \file advantage.cpp
 * \brief Advantage learning predictor source file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-10-05
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

#include <grl/predictors/advantage.h>

using namespace grl;

REGISTER_CONFIGURABLE(AdvantagePredictor)

void AdvantagePredictor::request(ConfigurationRequest *config)
{
  Predictor::request(config);
  
  config->push_back(CRP("alpha", "Learning rate", alpha_));
  config->push_back(CRP("gamma", "Discount rate", gamma_));
  config->push_back(CRP("lambda", "Trace decay rate", lambda_));
  config->push_back(CRP("kappa", "Advantage scaling factor", kappa_));

  config->push_back(CRP("discretizer", "discretizer.action", "Action discretizer", discretizer_));
  config->push_back(CRP("projector", "projector.pair", "Projects observation-action pairs onto representation space", projector_));
  config->push_back(CRP("representation", "representation.value/action", "A-value representation", representation_));
  config->push_back(CRP("trace", "trace", "Trace of projections", trace_, true));
}

void AdvantagePredictor::configure(Configuration &config)
{
  Predictor::configure(config);
  
  discretizer_ = (Discretizer*)config["discretizer"].ptr();

  projector_ = (Projector*)config["projector"].ptr();
  representation_ = (Representation*)config["representation"].ptr();
  trace_ = (Trace*)config["trace"].ptr();
  
  alpha_ = config["alpha"];
  gamma_ = config["gamma"];
  lambda_ = config["lambda"];
  kappa_ = config["kappa"];
}

void AdvantagePredictor::reconfigure(const Configuration &config)
{
  Predictor::reconfigure(config);
  
  if (config.has("action") && config["action"].str() == "reset")
    finalize();
}

void AdvantagePredictor::update(const Transition &transition)
{
  Predictor::update(transition);

  std::vector<Vector> variants;
  std::vector<ProjectionPtr> actions;
  Vector value;

  // A(x_t, u_t)
  ProjectionPtr p = projector_->project(transition.prev_obs, transition.prev_action);
  double a = representation_->read(p, &value);
  
  // Find max_u A(x_t, u)
  discretizer_->options(transition.prev_obs, &variants);
  projector_->project(transition.prev_obs, variants, &actions);
  double v=-std::numeric_limits<double>::infinity();
  for (size_t kk=0; kk < variants.size(); ++kk)
    v = fmax(v, representation_->read(actions[kk], &value));
    
  double target = v + (transition.reward - v)/kappa_;
  
  if (transition.action.size())
  {
    // Find max_u A(x_{t+1}, u)
    discretizer_->options(transition.obs, &variants);
    projector_->project(transition.obs, variants, &actions);
    v=-std::numeric_limits<double>::infinity();
    for (size_t kk=0; kk < variants.size(); ++kk)
      v = fmax(v, representation_->read(actions[kk], &value));
      
    target += gamma_*v/kappa_;
  }          
  
  double delta = target - a;
  
  representation_->write(p, VectorConstructor(target), alpha_);
  
  // TODO: Should clear trace on exploration
  if (trace_)
  {
    representation_->update(*trace_, VectorConstructor(alpha_*delta), gamma_*lambda_);
    trace_->add(p, gamma_*lambda_);
  }
  
  representation_->finalize();
}

void AdvantagePredictor::finalize()
{
  Predictor::finalize();
  
  if (trace_)
    trace_->clear();
}
