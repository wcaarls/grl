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
  config->push_back(CRP("alpha", "Learning rate", alpha_));
  config->push_back(CRP("eta", "Relative secondary learning rate (actual is alpha*eta)", eta_));
  config->push_back(CRP("gamma", "Discount rate", gamma_));

  config->push_back(CRP("projector", "projector", "Projects observation-action pairs onto representation space", projector_));
  config->push_back(CRP("representation", "representation", "(Q, w) representation", representation_));
  config->push_back(CRP("policy", "policy/discrete/q", "Greedy target policy", policy_));
}

void GGQPredictor::configure(Configuration &config)
{
  projector_ = (Projector*)config["projector"].ptr();
  representation_ = (Representation*)config["representation"].ptr();
  policy_ = (QPolicy*)config["policy"].ptr();
  
  alpha_ = config["alpha"];
  eta_ = config["eta"];
  gamma_ = config["gamma"];
}

void GGQPredictor::reconfigure(const Configuration &config)
{
}

GGQPredictor *GGQPredictor::clone() const
{
  return NULL;
}

void GGQPredictor::update(const Transition &transition)
{
  Vector v;
  
  // phi (actual taken action)
  ProjectionPtr phi = projector_->project(transition.prev_obs, transition.prev_action);
  
  // phi_next for greedy target policy
  Vector action;
  policy_->act(transition.prev_obs, transition.prev_action, transition.obs, &action);
  ProjectionPtr phi_next = projector_->project(transition.obs, action);

  // temporal difference error
  double target = transition.reward + gamma_*representation_->read(phi_next, &v);
  double delta = target - representation_->read(phi, &v);
  
  // w^Tphi
  double dotwphi = 0.;
  if (!v.empty())
   dotwphi = v[1];

  // Update weights
  representation_->write(phi, VectorConstructor(target, delta), VectorConstructor(alpha_, alpha_*eta_));
  representation_->update(phi_next, VectorConstructor(-alpha_*gamma_*dotwphi, 0.));
}

void GGQPredictor::finalize()
{
}
