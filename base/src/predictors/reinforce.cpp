/** \file reinforce.cpp
 * \brief Monte-Carlo Policy Gradient predictor source file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2017-05-09
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

#include <grl/predictors/reinforce.h>

using namespace grl;

REGISTER_CONFIGURABLE(ReinforcePredictor)

void ReinforcePredictor::request(ConfigurationRequest *config)
{
  Predictor::request(config);

  config->push_back(CRP("alpha", "learning rate", alpha_));
  config->push_back(CRP("gamma", "Discount rate", gamma_));

  config->push_back(CRP("projector", "projector.observation", "Projects observations onto representation space", projector_));
  config->push_back(CRP("representation", "representation.action", "Policy representation", representation_));
}

void ReinforcePredictor::configure(Configuration &config)
{
  Predictor::configure(config);
  
  projector_ = (Projector*)config["projector"].ptr();
  representation_ = (Representation*)config["representation"].ptr();
  
  alpha_ = config["alpha"];
  gamma_ = config["gamma"];
}

void ReinforcePredictor::reconfigure(const Configuration &config)
{
  Predictor::reconfigure(config);
  
  if (config.has("action") && config["action"].str() == "reset")
    transitions_.clear();
}

void ReinforcePredictor::update(const Transition &transition)
{
  Predictor::update(transition);

  transitions_.push_back(transition);
}

void ReinforcePredictor::finalize()
{
  Predictor::finalize();
  
  double r = 0;
  for (int ii = transitions_.size()-1; ii >= 0; --ii)
  {
    const Transition &t=transitions_[ii];
    ProjectionPtr p = projector_->project(t.prev_obs);
    Vector a;
    
    r = gamma_*r + t.reward;
    
    representation_->read(p, &a);
    representation_->update(p, alpha_*(t.prev_action.v-a)*r);
  }
  
  transitions_.clear();
}
