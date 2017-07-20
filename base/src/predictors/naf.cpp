/** \file naf.cpp
 * \brief NAF predictor source file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2017-07-18
 *
 * \copyright \verbatim
 * Copyright (c) 2017, Wouter Caarls
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

#include <grl/predictors/naf.h>

using namespace grl;

REGISTER_CONFIGURABLE(NAFPredictor)

void NAFPredictor::request(ConfigurationRequest *config)
{
  Predictor::request(config);
  
  config->push_back(CRP("gamma", "Discount rate", gamma_));

  config->push_back(CRP("projector", "projector.pair", "Projects observation-action pairs onto representation space", projector_));
  config->push_back(CRP("representation", "representation.value/action", "Combined (action, state value) representation", representation_));
  config->push_back(CRP("target_representation", "representation.value/action", "Representation for calculating targets", target_representation_));
}

void NAFPredictor::configure(Configuration &config)
{
  Predictor::configure(config);
  
  projector_ = (Projector*)config["projector"].ptr();
  representation_ = (Representation*)config["representation"].ptr();
  target_representation_ = (Representation*)config["target_representation"].ptr();
  
  gamma_ = config["gamma"];
}

void NAFPredictor::reconfigure(const Configuration &config)
{
  Predictor::reconfigure(config);
}

void NAFPredictor::update(const std::vector<const Transition*> &transitions)
{
  Matrix v;
  size_t v_idx = transitions[0]->prev_action.size();
  
  size_t vs = 0;
  for (size_t ii=0; ii < transitions.size(); ++ii)
    if (!transitions[ii]->obs.absorbing)
      vs++;
  
  target_representation_->batchRead(vs);
  for (size_t ii=0; ii < transitions.size(); ++ii)
    if (!transitions[ii]->obs.absorbing)
    {
      target_representation_->enqueue(projector_->project(transitions[ii]->obs, transitions[ii]->action));
    }
  target_representation_->read(&v);
  
  vs = 0;
  representation_->batchWrite(transitions.size());
  for (size_t ii=0; ii < transitions.size(); ++ii)
  {
    double target = transitions[ii]->reward;
  
    if (!transitions[ii]->obs.absorbing)
      target += gamma_*v(vs++, v_idx);
      
    representation_->enqueue(projector_->project(transitions[ii]->prev_obs, transitions[ii]->prev_action), VectorConstructor(target));
  }
  representation_->write();
}

void NAFPredictor::finalize()
{
  Predictor::finalize();
}
