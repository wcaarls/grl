/** \file ddpg.cpp
 * \brief DDPG predictor source file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2018-04-14
 *
 * \copyright \verbatim
 * Copyright (c) 2018, Wouter Caarls
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

#include <grl/predictors/ddpg.h>

using namespace grl;

REGISTER_CONFIGURABLE(DDPGPredictor)

void DDPGPredictor::request(ConfigurationRequest *config)
{
  Predictor::request(config);
  
  config->push_back(CRP("gamma", "Discount rate", gamma_));
  config->push_back(CRP("reward_scale", "Scaling factor for all rewards", reward_scale_, CRP::Configuration, DBL_MIN, DBL_MAX));
  
  config->push_back(CRP("observation", "TensorFlow placeholder node for graph observation input", observation_));
  config->push_back(CRP("action", "TensorFlow node for graph action input", action_));
  config->push_back(CRP("value", "TensorFlow operation output to read Q value", target_));
  config->push_back(CRP("target", "TensorFlow placeholder node to set target Q value", target_));
  config->push_back(CRP("critic_update", "TensorFlow operation node to run critic update", critic_update_));
  config->push_back(CRP("actor_update", "TensorFlow operation node to run actor update", actor_update_));

  config->push_back(CRP("discretizer", "discretizer.action", "Optional action discretizer to take max over next state", discretizer_, true));
  config->push_back(CRP("obs_projector", "projector.observation", "Projects observation onto representation space", obs_projector_));
  config->push_back(CRP("action_projector", "projector.action", "Projects action onto representation space", action_projector_));
  config->push_back(CRP("representation", "representation/parameterized/tensorflow.action", "Action representation trained with Q targets", representation_));
}

void DDPGPredictor::configure(Configuration &config)
{
  Predictor::configure(config);
  
  observation_ = config["observation"].str();
  action_ = config["action"].str();
  value_ = config["value"].str();
  target_ = config["target"].str();
  critic_update_ = config["critic_update"].str();
  actor_update_ = config["actor_update"].str();
  
  discretizer_ = (Discretizer*)config["discretizer"].ptr();
  obs_projector_ = (Projector*)config["obs_projector"].ptr();
  action_projector_ = (Projector*)config["action_projector"].ptr();
  representation_ = (TensorFlowRepresentation*)config["representation"].ptr();
  
  gamma_ = config["gamma"];
  reward_scale_ = config["reward_scale"];
}

void DDPGPredictor::reconfigure(const Configuration &config)
{
  Predictor::reconfigure(config);
}

void DDPGPredictor::update(const std::vector<const Transition*> &transitions)
{
  // Count number of transitions with valid next state  
  long int qs = 0, as = 0;
  for (size_t ii=0; ii < transitions.size(); ++ii)
    if (!transitions[ii]->obs.absorbing)
      qs++;
      
  if (discretizer_)
    as = discretizer_->size();
  
  // Verify projectors
  ProjectionPtr p_obs = obs_projector_->project(transitions[0]->prev_obs);
  VectorProjection *vp_obs = dynamic_cast<VectorProjection*>(p_obs.get());
  if (!vp_obs)
    throw Exception("DDPG predictor requires vector observation projection");
    
  ProjectionPtr p_action = action_projector_->project(transitions[0]->prev_action);
  VectorProjection *vp_action = dynamic_cast<VectorProjection*>(p_action.get());
  if (!vp_action)
    throw Exception("DDPG predictor requires vector action projection");
    
  // Create input tensor
  TF::TensorPtr read_input = representation_->tensor(TF::Shape({qs, vp_obs->vector.size()}));
  
  qs = 0;
  for (size_t ii=0; ii < transitions.size(); ++ii)
    if (!transitions[ii]->obs.absorbing)
    {
      ProjectionPtr projection = obs_projector_->project(transitions[ii]->obs);
      VectorProjection *vp = dynamic_cast<VectorProjection*>(projection.get());
      for (size_t jj=0; jj < vp->vector.size(); ++jj)
        (*read_input)(qs, jj) = vp->vector[jj];
      qs++;
    }
  
  // Get Q(s', a')
  std::vector<TF::TensorPtr> result;
  representation_->target()->SessionRun({{observation_, read_input}}, {value_}, {}, &result);

  TF::Tensor &qap = *result[0];
  size_t networks = 1;
  if (qap.shape().num_dims() > 1)
    networks = qap.shape().dims()[1];
    
  std::vector<TF::TensorPtr> disc_result;
  if (discretizer_)
  {
    // Create input tensor for discrete actions
    TF::TensorPtr disc_input_obs = representation_->tensor(TF::Shape({qs*as, vp_obs->vector.size()})),
                  disc_input_act = representation_->tensor(TF::Shape({qs*as, vp_action->vector.size()}));
    
    qs = 0;
    for (size_t ii=0; ii < transitions.size(); ++ii)
      if (!transitions[ii]->obs.absorbing)
      {
        std::vector<Vector> variants;
        discretizer_->options(transitions[ii]->obs, &variants);
      
        for (size_t aa=0; aa < as; ++aa)
        {
          for (size_t jj=0; jj < vp_obs->vector.size(); ++jj)
            (*disc_input_obs)(qs*as+aa, jj) = (*read_input)(qs, jj);
            
          ProjectionPtr projection = action_projector_->project(variants[aa]);
          VectorProjection *vp = dynamic_cast<VectorProjection*>(projection.get());
          for (size_t jj=0; jj < vp->vector.size(); ++jj)
            (*disc_input_act)(qs*as+aa, jj) = vp->vector[jj];
        }
        
        qs++;
      }
  
    // Get Q(s', {variants})
    representation_->target()->SessionRun({{observation_, disc_input_obs}, {action_, disc_input_act}}, {value_}, {}, &disc_result);
  }
  
  TF::TensorPtr write_obs_input = representation_->tensor(TF::Shape({(int)transitions.size(), vp_obs->vector.size()}));
  TF::TensorPtr write_action_input = representation_->tensor(TF::Shape({(int)transitions.size(), vp_action->vector.size()}));
  TF::TensorPtr write_target = representation_->tensor(TF::Shape({(int)transitions.size(), (int)networks}));
  
  // Create target tensor 
  qs = 0;
  
  for (size_t ii=0; ii < transitions.size(); ++ii)
  {
    ProjectionPtr obs_projection = obs_projector_->project(transitions[ii]->prev_obs);
    VectorProjection *vp_obs = dynamic_cast<VectorProjection*>(obs_projection.get());
    for (size_t jj=0; jj < vp_obs->vector.size(); ++jj)
      (*write_obs_input)(ii, jj) = vp_obs->vector[jj];
      
    ProjectionPtr action_projection = action_projector_->project(transitions[ii]->prev_action);
    VectorProjection *vp_action = dynamic_cast<VectorProjection*>(action_projection.get());
    for (size_t jj=0; jj < vp_action->vector.size(); ++jj)
      (*write_action_input)(ii, jj) = vp_action->vector[jj];

    for (size_t nn=0; nn < networks; ++nn)
    {
      double target = reward_scale_*transitions[ii]->reward;
  
      if (!transitions[ii]->obs.absorbing)
      {
        double v = qap(qs, nn);
        
        if (discretizer_)
          for (size_t aa=0; aa < discretizer_->size(); ++aa)
            v = fmax(v, (*disc_result[0])(qs*as+aa, nn));

        target += pow(gamma_, transitions[ii]->tau)*v;
      }
       
      (*write_target)(ii, nn) = target;
    }
    
    if (!transitions[ii]->obs.absorbing)
      qs++;
  }
  
  // Update critic
  representation_->SessionRun({{observation_, write_obs_input}, {action_, write_action_input}, {target_, write_target}}, {}, {critic_update_}, &result, true);

#if VALS
  // Get Q(s, a)
  std::vector<TF::TensorPtr> before;
  representation_->SessionRun({{observation_, write_obs_input}}, {value_, action_}, {}, &before, false);
  TF::Tensor &before_q = *before[0];
  TF::Tensor &before_a = *before[1];
#endif
  
  // Update actor
  representation_->SessionRun({{observation_, write_obs_input}}, {}, {actor_update_}, &result, true);

#if VALS
  // Get Q(s, a)
  std::vector<TF::TensorPtr> after;
  representation_->SessionRun({{observation_, write_obs_input}}, {value_, action_}, {}, &after, false);
  TF::Tensor &after_q = *after[0];
  TF::Tensor &after_a = *after[1];
  
  double diff = 0;
  for (size_t ii=0; ii < transitions.size(); ++ii)
    diff += after_q(ii) - before_q(ii);
  NOTICE("Sum Q update from actor learning step: " << diff);
#endif
}

void DDPGPredictor::finalize()
{
  Predictor::finalize();
}
