/** \file vi.cpp
 * \brief Deterministic value iteration backup predictors header file.
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

#include <iomanip>

#include <grl/predictors/vi.h>

using namespace grl;

REGISTER_CONFIGURABLE(ValueIterationPredictor)
REGISTER_CONFIGURABLE(QIterationPredictor)

void ValueIterationPredictor::request(ConfigurationRequest *config)
{
  Predictor::request(config);
  
  config->push_back(CRP("gamma", "Discount rate", gamma_));
  
  config->push_back(CRP("model", "observation_model", "Observation model used for planning", model_));
  config->push_back(CRP("discretizer", "discretizer.action", "Action discretizer", discretizer_));
  config->push_back(CRP("projector", "projector.observation", "Projects observations onto representation space", projector_));
  config->push_back(CRP("representation", "representation.value/state", "State-value function representation", representation_));
}

void ValueIterationPredictor::configure(Configuration &config)
{
  Predictor::configure(config);
  
  gamma_ = config["gamma"];
  
  model_ = (ObservationModel*)config["model"].ptr();
  discretizer_ = (Discretizer*)config["discretizer"].ptr();

  projector_ = (Projector*)config["projector"].ptr();
  representation_ = (Representation*)config["representation"].ptr();
}

void ValueIterationPredictor::reconfigure(const Configuration &config)
{
  Predictor::reconfigure(config);
}

void ValueIterationPredictor::update(const Transition &transition)
{
  Predictor::update(transition);
  
  double v=-std::numeric_limits<double>::infinity();
  
  for (Discretizer::iterator it = discretizer_->begin(transition.prev_obs); it != discretizer_->end(); ++it)
  {
    Vector next;
    double reward;
    int terminal;
    model_->step(transition.prev_obs, *it, &next, &reward, &terminal);
    
    if (next.size())
    {
      if (!terminal)
      {
        Vector value;
        reward += gamma_*representation_->read(projector_->project(next), &value);
      }
      
      v = fmax(v, reward);
    }
  }

  if (std::isfinite(v))
    representation_->write(projector_->project(transition.prev_obs), VectorConstructor(v));
}

void QIterationPredictor::request(ConfigurationRequest *config)
{
  Predictor::request(config);
  
  config->push_back(CRP("gamma", "Discount rate", gamma_));
  
  config->push_back(CRP("model", "observation_model", "Observation model used for planning", model_));
  config->push_back(CRP("discretizer", "discretizer.action", "Action discretizer", discretizer_));
  config->push_back(CRP("projector", "projector.pair", "Projects observation-action pairs onto representation space", projector_));
  config->push_back(CRP("representation", "representation.value/action", "Action-value function representation", representation_));
}

void QIterationPredictor::update(const Transition &transition)
{
  Predictor::update(transition);
  
  for (Discretizer::iterator it=discretizer_->begin(transition.prev_obs); it != discretizer_->end(); ++it)
  {
    Vector next;
    double reward;
    int terminal;
    model_->step(transition.prev_obs, *it, &next, &reward, &terminal);
    
    if (next.size())
    {
      if (!terminal)
      {
        // Find value of best action
        Vector value;
        double v=-std::numeric_limits<double>::infinity();
        for (Discretizer::iterator it2=discretizer_->begin(next); it2 != discretizer_->end(); ++it2)
          v = fmax(v, representation_->read(projector_->project(next, *it2), &value));
        
        reward += gamma_*v;
      }
    
      representation_->write(projector_->project(transition.prev_obs, *it), VectorConstructor(reward));
    }
  }
}
