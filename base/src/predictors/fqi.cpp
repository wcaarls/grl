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

  std::vector<std::string> options;
  options.push_back("never");
  options.push_back("batch");
  options.push_back("iteration");
  config->push_back(CRP("reset_strategy", "At which point to reset the representation", reset_strategy_str_, CRP::Configuration, options));

  config->push_back(CRP("discretizer", "discretizer.action", "Action discretizer", discretizer_));
  config->push_back(CRP("projector", "projector.pair", "Projects observations onto critic representation space", projector_));
  config->push_back(CRP("representation", "representation.value/action", "Value function representation", representation_));
}

void FQIPredictor::configure(Configuration &config)
{
  discretizer_ = (Discretizer*)config["discretizer"].ptr();
  discretizer_->options(&variants_);
  
  projector_ = (Projector*)config["projector"].ptr();
  representation_ = (Representation*)config["representation"].ptr();
  
  gamma_ = config["gamma"];
  max_samples_ = config["transitions"];
  iterations_ = config["iterations"];
  
  reset_strategy_str_ = config["reset_strategy"].str();
  if (reset_strategy_str_ == "never")          reset_strategy_ = rsNever;
  else if (reset_strategy_str_ == "batch")     reset_strategy_ = rsBatch;
  else if (reset_strategy_str_ == "iteration") reset_strategy_ = rsIteration;
  else throw bad_param("predictor/fqi:reset_strategy");

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
  fqp->discretizer_ = discretizer_->clone();
  fqp->projector_ = projector_->clone();
  fqp->representation_ = representation_->clone();
  return fqp;
}

void FQIPredictor::update(const Transition &transition)
{
  transitions_.push_back(CachedTransition(transition));
}

void FQIPredictor::finalize()
{
  std::vector<double> targets(transitions_.size(), 0.);
  double maxdelta = std::numeric_limits<double>::infinity();
  
  if (reset_strategy_ >= rsBatch)
    representation_->reset();
    
  for (size_t ii=0; ii < iterations_ && maxdelta > 0.001; ++ii)
  {
    bool reproject_actions = projector_->lifetime() == Projector::plUpdate ||
                             (projector_->lifetime() == Projector::plWrite && (ii < 2 || reset_strategy_ == rsIteration));

    bool reproject_state = projector_->lifetime() >= Projector::plWrite &&
                           reset_strategy_ >= rsBatch &&
                           (ii == 0 || reset_strategy_ == rsIteration);
        
    maxdelta = 0;
    
    #pragma omp parallel default(shared)
    {
      double threadmax = 0;
  
      // Convert to sample set
      #ifdef _OPENMP
      #pragma omp for schedule(static)
      #endif
      for (size_t jj=0; jj < transitions_.size(); ++jj)
      {
        CachedTransition& t = transitions_[jj];
        double target = t.transition.reward;

        if (!t.transition.obs.empty())
        {
          if (t.actions.empty() || reproject_actions)
          {
            // Rebuild next state-action projections
            t.actions.clear();
            projector_->project(t.transition.obs, variants_, &t.actions);
          }
          
          // Find value of best action
          Vector value;
          double v=-std::numeric_limits<double>::infinity();
          for (size_t kk=0; kk < variants_.size(); ++kk)
            v = fmax(v, representation_->read(t.actions[kk], &value));
       
          target += gamma_*v;
        }
        
        threadmax = fmax(threadmax, fabs(targets[jj]-target));
        targets[jj] = target;
      }
      
      #pragma omp critical
      maxdelta = fmax(maxdelta, threadmax);
    }
    
    if (reset_strategy_ == rsIteration)
      representation_->reset();
    
    // Learn
    for (size_t jj=0; jj < transitions_.size(); ++jj)
    {
      CachedTransition& t = transitions_[jj];
      
      if (!t.state || reproject_state)
        t.state = projector_->project(t.transition.prev_obs, t.transition.prev_action);
      
      representation_->write(t.state, VectorConstructor(targets[jj]));
    }
    
    representation_->finalize();
    
    CRAWL("FQI iteration " << ii << " delta L_inf: " << maxdelta);
  }
}
