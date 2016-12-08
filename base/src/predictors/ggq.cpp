/** \file ggq.cpp
 * \brief Greedy-GQ predictor source file.
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

#include <grl/predictors/ggq.h>

using namespace grl;

REGISTER_CONFIGURABLE(GGQPredictor)

void GGQPredictor::request(ConfigurationRequest *config)
{
  Predictor::request(config);

  config->push_back(CRP("alpha", "Learning rate", alpha_));
  config->push_back(CRP("eta", "Relative secondary learning rate (actual is alpha*eta)", eta_));
  config->push_back(CRP("gamma", "Discount rate", gamma_));

  config->push_back(CRP("projector", "projector.pair", "Projects observation-action pairs onto representation space", projector_));
  config->push_back(CRP("representation", "representation.value/action", "(Q, w) representation", representation_));
  config->push_back(CRP("policy", "policy/discrete/q", "Greedy target policy", policy_));
}

void GGQPredictor::configure(Configuration &config)
{
  Predictor::configure(config);
  
  projector_ = (Projector*)config["projector"].ptr();
  representation_ = (Representation*)config["representation"].ptr();
  policy_ = (QPolicy*)config["policy"].ptr();
  
  alpha_ = config["alpha"];
  eta_ = config["eta"];
  gamma_ = config["gamma"];
}

void GGQPredictor::reconfigure(const Configuration &config)
{
  Predictor::reconfigure(config);
  
}

void GGQPredictor::update(const Transition &transition)
{
  Predictor::update(transition);

  Vector v;
  
  // phi (actual taken action)
  ProjectionPtr phi = projector_->project(transition.prev_obs, transition.prev_action), phi_next;
  double target = transition.reward;
  
  // phi_next for greedy target policy
  if (transition.action.size())
  {
    Vector action;
    policy_->act(transition.obs, &action);
    phi_next = projector_->project(transition.obs, action);
    target += gamma_*representation_->read(phi_next, &v);
  }

  // temporal difference error
  double delta = target - representation_->read(phi, &v);
  
  // w^Tphi is the second output of the representation
  double dotwphi = 0.;
  if (v.size())
  {
    if (v.size() < 2)
    {
      ERROR("GGQ predictor requires a representation with at least two outputs");
      throw bad_param("predictor/ggq:representation");
    }
    else
      dotwphi = v[1];
  }

  // Update weights
  representation_->write(phi, VectorConstructor(target, delta), VectorConstructor(alpha_, alpha_*eta_));
  
  if (transition.action.size())
    representation_->update(phi_next, VectorConstructor(-alpha_*gamma_*dotwphi, 0.));
    
  representation_->finalize();
}

void GGQPredictor::finalize()
{
  Predictor::finalize();
}
