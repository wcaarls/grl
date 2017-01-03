/** \file mbfqi.cpp
 * \brief Minibatch MBFQI predictor source file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2017-01-02
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

#include <grl/predictors/mbfqi.h>

using namespace grl;

REGISTER_CONFIGURABLE(MBFQIPredictor)

void MBFQIPredictor::request(ConfigurationRequest *config)
{
  Predictor::request(config);

  config->push_back(CRP("alpha", "Learning rate", alpha_));
  config->push_back(CRP("gamma", "Discount rate", gamma_));
  config->push_back(CRP("transitions", "Maximum number of transitions to store", max_samples_, CRP::Configuration, 1));

  config->push_back(CRP("minibatch_size", "Number of transitions to average gradient over.", (int)minibatch_size_));
  config->push_back(CRP("update_interval", "Number of minibatches between target updates.", (int)update_interval_, CRP::Configuration, 1));

  config->push_back(CRP("target", "mapping", "Target value at next state", target_));
  config->push_back(CRP("projector", "projector.pair", "Projects observations onto critic representation space", projector_));
  config->push_back(CRP("representation", "representation.value/action", "Value function representation", representation_));
}

void MBFQIPredictor::configure(Configuration &config)
{
  Predictor::configure(config);
  
  target_ = (Mapping*)config["target"].ptr();
  projector_ = (Projector*)config["projector"].ptr();
  representation_ = (Representation*)config["representation"].ptr();
  
  alpha_ = config["alpha"];
  gamma_ = config["gamma"];
  max_samples_ = config["transitions"];
  minibatch_size_ = config["minibatch_size"];
  update_interval_ = config["update_interval"];
  
  // Initialize memory
  reset();
}

void MBFQIPredictor::reconfigure(const Configuration &config)
{
  Predictor::reconfigure(config);
  
  if (config.has("action") && config["action"].str() == "reset")
  {
    TRACE("Initializing transition store");
  
    transitions_.clear();
    indices_.clear();
    update_counter_ = 0;
  }
}

MBFQIPredictor &MBFQIPredictor::copy(const Configurable &obj)
{
  const MBFQIPredictor& fp = dynamic_cast<const MBFQIPredictor&>(obj);
  
  transitions_ = fp.transitions_;
  indices_ = fp.indices_;
  update_counter_ = fp.update_counter_;

  return *this;
}

void MBFQIPredictor::update(const Transition &transition)
{
  Predictor::update(transition);

  // Add new transition to replay memory
  transitions_.push_back(CachedTransition(transition));
  CachedTransition& t = transitions_[transitions_.size()-1];
  
  // If the projector has infinite lifetime, we only need to project once
  if (projector_->lifetime() == Projector::plIndefinite)
    t.projection = projector_->project(t.transition.prev_obs, t.transition.prev_action);
    
  // Before first rebuild, use immediate rewards
  t.target = t.transition.reward;
    
  // Accumulate gradients
  // ANNRepresentation is not reentrant, so we don't bother parallelizing this loop.
  for (size_t jj=0; jj < minibatch_size_; ++jj)
  {
    CachedTransition& t = indices_.size()?transitions_[indices_[update_counter_*minibatch_size_+jj]]
                                         :transitions_[RandGen::getInteger(transitions_.size())];
    
    ProjectionPtr p;
    
    if (projector_->lifetime() != Projector::plIndefinite)
      p = projector_->project(t.transition.prev_obs, t.transition.prev_action);
    else
      p = t.projection;

    // Alpha is for representations that don't have an internal learning rate, such as tile coding.
    representation_->write(p, VectorConstructor(t.target), alpha_);
  }

  // Update representation
  representation_->finalize();

  // Recalculate targets if necessary
  if (++update_counter_ == update_interval_)
  {
    rebuild();
    update_counter_ = 0;
  }
}

void MBFQIPredictor::finalize()
{
  Predictor::finalize();
}

void MBFQIPredictor::rebuild()
{
  indices_.resize(minibatch_size_ * update_interval_);
  
  // We're avoiding calculating the target of the same transition twice here,
  // but that is not really relevant if there are many more transitions
  // than updates between rebuilds.
  
  #ifdef _OPENMP
  #pragma omp parallel for schedule(static)
  #endif
  for (size_t jj=0; jj < transitions_.size(); ++jj)
    transitions_[jj].target = -std::numeric_limits<double>::infinity();

  #ifdef _OPENMP
  #pragma omp parallel
  #endif
  {
    // Avoid calling pthread_getspecific every iteration
    Rand *rand = RandGen::instance();
    
    #pragma omp for
    for (size_t jj=0; jj < minibatch_size_ * update_interval_; ++jj)
    {
      size_t idx = rand->getInteger(transitions_.size());
      
      CachedTransition& t = transitions_[idx];
      
      if (t.target == -std::numeric_limits<double>::infinity())
      {
        double target = t.transition.reward;

        Vector v;
        if (t.transition.action.size())
          target += gamma_ * target_->read(t.transition.obs, &v);
      
        t.target = target;
      }
      
      indices_[jj] = idx;
    }
  }
}
