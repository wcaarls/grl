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
  
  config->push_back(CRP("input", "TensorFlow placeholder node for graph input", input_));
  config->push_back(CRP("value", "TensorFlow operation output to read state-value", input_));
  config->push_back(CRP("target", "TensorFlow placeholder node to set target action-value", input_));
  config->push_back(CRP("update", "TensorFlow operation node to run update", input_));

  config->push_back(CRP("projector", "projector.pair", "Projects observation-action pairs onto representation space", projector_));
  config->push_back(CRP("representation", "representation/parameterized/tensorflow.action", "Action representation with extra node for state-value retrieval and action-value update", representation_));
}

void NAFPredictor::configure(Configuration &config)
{
  Predictor::configure(config);
  
  input_ = config["input"].str();
  value_ = config["value"].str();
  target_ = config["target"].str();
  update_ = config["update"].str();
  
  projector_ = (Projector*)config["projector"].ptr();
  representation_ = (TensorFlowRepresentation*)config["representation"].ptr();
  
  gamma_ = config["gamma"];
}

void NAFPredictor::reconfigure(const Configuration &config)
{
  Predictor::reconfigure(config);
}

void NAFPredictor::update(const std::vector<const Transition*> &transitions)
{
  // Count number of transitions with valid next state  
  long int vs = 0;
  for (size_t ii=0; ii < transitions.size(); ++ii)
    if (!transitions[ii]->obs.absorbing)
      vs++;
  
  // Verify projector
  ProjectionPtr projection = projector_->project(transitions[0]->prev_obs, transitions[0]->prev_action);
  VectorProjection *vp = dynamic_cast<VectorProjection*>(projection.get());
  if (!vp)
    throw Exception("NAF predictor requires vector projection");

  TF::TensorPtr read_input = representation_->tensor(TF::Shape({vs, vp->vector.size()}));
  
  // Create input tensor
  vs = 0;
  for (size_t ii=0; ii < transitions.size(); ++ii)
    if (!transitions[ii]->obs.absorbing)
    {
      ProjectionPtr projection = projector_->project(transitions[ii]->obs, transitions[ii]->action);
      VectorProjection *vp = dynamic_cast<VectorProjection*>(projection.get());
      for (size_t jj=0; jj < vp->vector.size(); ++jj)
        (*read_input)(vs, jj) = vp->vector[jj];
      vs++;
    }
  
  // Get V(s')
  std::vector<TF::TensorPtr> result;
  representation_->target()->SessionRun({{input_, read_input}}, {value_}, {}, &result);

  TF::Tensor &v = *result[0];
  TF::TensorPtr write_input = representation_->tensor(TF::Shape({vs, vp->vector.size()}));
  TF::TensorPtr write_target = representation_->tensor(TF::Shape({vs, 1}));
  
  // Create target tensor 
  vs = 0;
  for (size_t ii=0; ii < transitions.size(); ++ii)
  {
    double target = transitions[ii]->reward;
  
    if (!transitions[ii]->obs.absorbing)
    {
      TRACE("v: " << v(vs));
      target += gamma_*v(vs++);
    }
     
    ProjectionPtr projection = projector_->project(transitions[ii]->prev_obs, transitions[ii]->prev_action);
    VectorProjection *vp = dynamic_cast<VectorProjection*>(projection.get());
    for (size_t jj=0; jj < vp->vector.size(); ++jj)
      (*write_input)(ii, jj) = vp->vector[jj];
    
    (*write_target)(ii) = target;
  }
  
  // Write (s, a) -> r + \gamma*V(s')
  representation_->SessionRun({{input_, write_input}, {target_, write_target}}, {}, {update_}, &result, true);
}

void NAFPredictor::finalize()
{
  Predictor::finalize();
}
