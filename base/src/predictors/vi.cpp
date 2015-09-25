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
  discretizer_->options(&variants_);

  projector_ = (Projector*)config["projector"].ptr();
  representation_ = (Representation*)config["representation"].ptr();
}

void ValueIterationPredictor::reconfigure(const Configuration &config)
{
  Predictor::reconfigure(config);
}

ValueIterationPredictor *ValueIterationPredictor::clone() const
{
  ValueIterationPredictor *predictor = new ValueIterationPredictor(*this);
  
  predictor->model_ = model_->clone();
  predictor->discretizer_ = discretizer_->clone();
  predictor->projector_ = projector_->clone();
  predictor->representation_ = representation_->clone();
  
  return predictor;
}

void ValueIterationPredictor::update(const Transition &transition)
{
  Predictor::update(transition);
  
  double v=-std::numeric_limits<double>::infinity();
  
  for (size_t aa=0; aa < variants_.size(); ++aa)
  {
    Vector next;
    double reward;
    int terminal;
    model_->step(transition.prev_obs, variants_[aa], &next, &reward, &terminal);
    
    if (!next.empty())
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

QIterationPredictor *QIterationPredictor::clone() const
{
  QIterationPredictor *predictor = new QIterationPredictor(*this);
  
  predictor->model_ = model_->clone();
  predictor->discretizer_ = discretizer_->clone();
  predictor->projector_ = projector_->clone();
  predictor->representation_ = representation_->clone();
  
  return predictor;
}

void QIterationPredictor::update(const Transition &transition)
{
  Predictor::update(transition);
  
  for (size_t aa=0; aa < variants_.size(); ++aa)
  {
    Vector next;
    double reward;
    int terminal;
    model_->step(transition.prev_obs, variants_[aa], &next, &reward, &terminal);
    
    if (!next.empty())
    {
      if (!terminal)
      {
        // Find value of best action
        Vector value;
        double v=-std::numeric_limits<double>::infinity();
        for (size_t ii=0; ii < variants_.size(); ++ii)
          v = fmax(v, representation_->read(projector_->project(next, variants_[ii]), &value));
        
        reward += gamma_*v;
      }
    
      representation_->write(projector_->project(transition.prev_obs, variants_[aa]), VectorConstructor(reward));
    }
  }
}
