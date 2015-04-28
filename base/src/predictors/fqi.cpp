/** \file fqi.cpp
 * \brief Fitted Q-iteration predictor source file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-04-28
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

#include <grl/predictors/fqi.h>

using namespace grl;

REGISTER_CONFIGURABLE(FQIPredictor)

void FQIPredictor::request(ConfigurationRequest *config)
{
  config->push_back(CRP("gamma", "Discount rate", gamma_));
  config->push_back(CRP("transitions", "Maximum number of transitions to store", max_samples_, CRP::Configuration, 1));
  config->push_back(CRP("iterations", "Number of policy improvement rounds per episode", iterations_, CRP::Configuration, 1));

  config->push_back(CRP("projector", "projector.pair", "Projects observations onto critic representation space", projector_));
  config->push_back(CRP("representation", "representation.value/action", "Value function representation", representation_));
  config->push_back(CRP("policy", "policy/discrete/q", "Q-value based policy", policy_));
}

void FQIPredictor::configure(Configuration &config)
{
  projector_ = (Projector*)config["projector"].ptr();
  representation_ = (Representation*)config["representation"].ptr();
  policy_ = (QPolicy*)config["policy"].ptr();
  
  gamma_ = config["gamma"];
  max_samples_ = config["transitions"];
  iterations_ = config["iterations"];

  // Initialize memory
  reset();
}

void FQIPredictor::reconfigure(const Configuration &config)
{
  if (config.has("action") && config["action"].str() == "reset")
  {
    DEBUG("Initializing transition store");
  
    transitions_.clear();
  }
}

FQIPredictor *FQIPredictor::clone() const
{
  FQIPredictor *fqp = new FQIPredictor(*this);
  fqp->projector_ = projector_->clone();
  fqp->representation_ = representation_->clone();
  fqp->policy_ = policy_->clone();
  return fqp;
}

void FQIPredictor::update(const Transition &transition)
{
  transitions_.push_back(transition);
}

void FQIPredictor::finalize()
{
  std::vector<double> targets(transitions_.size(), 0.);
  double maxdelta = std::numeric_limits<double>::infinity();

  representation_->reset();
    
  for (size_t ii=0; ii < iterations_ && maxdelta > 0.001; ++ii)
  {
    maxdelta = 0;
  
    // Convert to sample set
    for (size_t jj=0; jj < transitions_.size(); ++jj)
    {
      const Transition& transition = transitions_[jj];
      double target = transition.reward;

      if (!transition.obs.empty())
      {
        Vector values, distribution;
        policy_->values(transition.obs, &values);
        policy_->distribution(transition.obs, &distribution);

        double v = 0.;
        for (size_t kk=0; kk < values.size(); ++kk)
          v += values[kk]*distribution[kk];
     
        target += gamma_*v;
      }
      
      maxdelta = fmax(maxdelta, fabs(targets[jj]-target));
      targets[jj] = target;
    }
    
    representation_->reset();
    
    // Learn
    for (size_t jj=0; jj < transitions_.size(); ++jj)
    {
      const Transition& transition = transitions_[jj];
      representation_->write(projector_->project(transition.prev_obs, transition.prev_action), VectorConstructor(targets[jj]));
    }
    
    CRAWL("FQI iteration " << ii << " delta L_inf: " << maxdelta);
  }
}
