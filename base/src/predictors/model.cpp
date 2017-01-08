/** \file model.cpp
 * \brief Model predictor source file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-02-23
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

#include <grl/predictors/model.h>

using namespace grl;

REGISTER_CONFIGURABLE(ModelPredictor)

void ModelPredictor::request(ConfigurationRequest *config)
{
  Predictor::request(config);
  
  config->push_back(CRP("differential", "vector.differential", "State dimensions for which to predict deltas", differential_, CRP::Configuration));
  config->push_back(CRP("wrapping", "vector.wrapping", "Wrapping boundaries", wrapping_));
  config->push_back(CRP("projector", "projector.pair", "Projector for transition model (|S|+|A| dimensions)", projector_));
  config->push_back(CRP("representation", "representation.transition", "Representation for transition model (|S|+2 dimensions)", representation_));
}

void ModelPredictor::configure(Configuration &config)
{
  Predictor::configure(config);
  
  projector_ = (Projector*)config["projector"].ptr();
  representation_ = (Representation*)config["representation"].ptr();
  
  differential_ = config["differential"].v();
  if (!differential_.size())
    throw bad_param("predictor/model:differential");
  
  wrapping_ = config["wrapping"].v();
}

void ModelPredictor::reconfigure(const Configuration &config)
{
  Predictor::reconfigure(config);
  
}

void ModelPredictor::update(const Transition &transition)
{
  Predictor::update(transition);

  Vector target;
  
  if (differential_.size() < transition.obs.size())
    differential_ = extend(differential_, ConstantVector(transition.obs.size()-differential_.size(), differential_[0]));
  
  if (wrapping_.size() < transition.obs.size())
    wrapping_ = extend(wrapping_, ConstantVector(transition.obs.size()-wrapping_.size(), 0.));
    
  if (transition.obs.size())
  {
    target = transition.obs;
    
    for (size_t ii=0; ii < target.size(); ++ii)
    {
      if (differential_[ii])
        target[ii] -= transition.prev_obs[ii];
    
      if (wrapping_[ii])
      {
        if (target[ii] > 0.5*wrapping_[ii])
          target[ii] -= wrapping_[ii];
        else if (target[ii] < -0.5*wrapping_[ii])
          target[ii] += wrapping_[ii];
      }
    }
    
    target = extend(target, VectorConstructor(transition.reward, transition.action.size()==0));
  }
  else
  {
    // Undefined absorbing state
    target = ConstantVector(transition.prev_obs.size()+2, 0.);
    target[target.size()-2] = transition.reward;
    target[target.size()-1] = 1.;
  }
  
  ProjectionPtr p = projector_->project(extend(transition.prev_obs, transition.prev_action));
  representation_->write(p, target);
  
  representation_->finalize();
}

void ModelPredictor::finalize()
{
  Predictor::finalize();
}
