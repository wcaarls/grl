/** \file td.cpp
 * \brief TD predictor source file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-08-28
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

#include <grl/predictors/td.h>

using namespace grl;

REGISTER_CONFIGURABLE(TDPredictor)

void TDPredictor::request(ConfigurationRequest *config)
{
  config->push_back(CRP("alpha", "Learning rate", alpha_));
  config->push_back(CRP("gamma", "Discount rate", gamma_));
  config->push_back(CRP("lambda", "Trace decay rate", lambda_));

  config->push_back(CRP("projector", "projector.observation", "Projects observations onto representation space", projector_));
  config->push_back(CRP("representation", "representation.value/state", "State value representation", representation_));
  config->push_back(CRP("trace", "trace", "Trace of projections", trace_));
}

void TDPredictor::configure(Configuration &config)
{
  projector_ = (Projector*)config["projector"].ptr();
  representation_ = (Representation*)config["representation"].ptr();
  trace_ = (Trace*)config["trace"].ptr();
  
  alpha_ = config["alpha"];
  gamma_ = config["gamma"];
  lambda_ = config["lambda"];
}

void TDPredictor::reconfigure(const Configuration &config)
{
  if (config.has("action") && config["action"].str() == "reset")
    finalize();
}

TDPredictor *TDPredictor::clone() const
{
  return NULL;
}

void TDPredictor::update(const Transition &transition)
{
  ProjectionPtr p = projector_->project(transition.obs);
  Vector v;

  double target = transition.reward;
  if (!transition.obs.empty())
    target += gamma_*representation_->read(projector_->project(transition.obs), &v);
  double delta = target - representation_->read(p, &v);

  representation_->write(p, VectorConstructor(target), alpha_);
  
  // TODO: recently added point is not in trace
  representation_->update(*trace_, VectorConstructor(alpha_*delta), gamma_*lambda_);
  trace_->add(p, gamma_*lambda_);
  
  representation_->finalize();
}

void TDPredictor::finalize()
{
  trace_->clear();
}
