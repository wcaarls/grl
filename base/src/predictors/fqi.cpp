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
  Predictor::request(config);

  config->push_back(CRP("gamma", "Discount rate", gamma_));
  config->push_back(CRP("transitions", "Maximum number of transitions to store", max_samples_, CRP::Configuration, 1));
  config->push_back(CRP("iterations", "Number of policy improvement rounds per episode", iterations_, CRP::Configuration, 1));

  std::vector<std::string> options;
  options.push_back("never");
  options.push_back("batch");
  options.push_back("iteration");
  config->push_back(CRP("reset_strategy", "At which point to reset the representation", reset_strategy_str_, CRP::Configuration, options));
  config->push_back(CRP("macro_batch_size", "Number of episodes/batches after which prediction is rebuilt. Use 0 for no rebuilds.", (int)macro_batch_size_));

  config->push_back(CRP("discretizer", "discretizer.action", "Action discretizer", discretizer_));
  config->push_back(CRP("projector", "projector.pair", "Projects observations onto critic representation space", projector_));
  config->push_back(CRP("representation", "representation.value/action", "Value function representation", representation_));

}

void FQIPredictor::configure(Configuration &config)
{
  Predictor::configure(config);
  
  discretizer_ = (Discretizer*)config["discretizer"].ptr();
  discretizer_->options(&variants_);
  
  projector_ = (Projector*)config["projector"].ptr();
  representation_ = (Representation*)config["representation"].ptr();
  
  gamma_ = config["gamma"];
  max_samples_ = config["transitions"];
  iterations_ = config["iterations"];
  macro_batch_size_ = config["macro_batch_size"];
  
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
  Predictor::reconfigure(config);
  
  if (config.has("action") && config["action"].str() == "reset")
  {
    DEBUG("Initializing transition store");
  
    transitions_.clear();
    macro_batch_counter_ = 0;
  }
  else if (config["action"].str() == "load")
  {
    std::string file = config["file"].str() + path() + ".dat";
    std::replace(file.begin(), file.end(), '/', '_');

    std::ifstream fileInStream;
    fileInStream.open(file.c_str(),  std::ofstream::in);
    if (!fileInStream.good())
      return;
    size_t count, count_obs, count_action;
    fileInStream.read(reinterpret_cast<char*>(&count),	sizeof(size_t));
    fileInStream.read(reinterpret_cast<char*>(&count_obs),	sizeof(size_t));
    fileInStream.read(reinterpret_cast<char*>(&count_action),	sizeof(size_t));
    
    NOTICE("Reading " << count << " transitions from " << file);
    
    transitions_.reserve(count);

    CachedTransition ctr;
    Transition &tr = ctr.transition;
    tr.prev_obs.resize(count_obs);
    tr.prev_action.resize(count_action);
    tr.obs.resize(count_obs);
    tr.action.resize(count_action);
    for (unsigned int i = 0; i < count; i++)
    {
      fileInStream.read(reinterpret_cast<char*>(&(tr.prev_obs[0])), tr.prev_obs.size()*sizeof(double));
      fileInStream.read(reinterpret_cast<char*>(&(tr.prev_action[0])), tr.prev_action.size()*sizeof(double));
      fileInStream.read(reinterpret_cast<char*>(&(tr.obs[0])), tr.obs.size()*sizeof(double));
      fileInStream.read(reinterpret_cast<char*>(&(tr.action[0])), tr.action.size()*sizeof(double));
      if (std::isnan(tr.obs[0]))
      { // this is a terminal or absorbing transition
        tr.obs.clear();
        tr.action.clear();
      }
      fileInStream.read(reinterpret_cast<char*>(&(tr.reward)), sizeof(double));

      transitions_.push_back(ctr);
      if (tr.obs.empty())
      { // restore
        tr.obs.resize(count_obs);
        tr.action.resize(count_action);
      }
    }
    fileInStream.close();
    
    rebuild(); // Build predictor
  }
  else if (config["action"].str() == "save")
  {
    if (transitions_.size() == 0)
      return;

    std::string file = config["file"].str() + path() + ".dat";
    std::replace(file.begin(), file.end(), '/', '_');

    std::ofstream fileOutStream;
    fileOutStream.open(file.c_str(),  std::ofstream::out);
    if (!fileOutStream.good())
      return;
    size_t count = transitions_.size();
    fileOutStream.write(reinterpret_cast<const char*>(&count),	sizeof(size_t));
    size_t count_obs = transitions_[0].transition.obs.size();
    fileOutStream.write(reinterpret_cast<const char*>(&count_obs),	sizeof(size_t));
    size_t count_action = transitions_[0].transition.action.size();
    fileOutStream.write(reinterpret_cast<const char*>(&count_action),	sizeof(size_t));
    // prepare dymmy symbols to obay format
    Vector dummy_obs, dummy_action;
    dummy_obs.resize(count_obs, std::numeric_limits<double>::signaling_NaN());
    dummy_action.resize(count_action, std::numeric_limits<double>::signaling_NaN());
    for (unsigned int i = 0; i < count; i++)
    {
      fileOutStream.write(reinterpret_cast<const char*>(&(transitions_[i].transition.prev_obs[0])), count_obs*sizeof(double));
      fileOutStream.write(reinterpret_cast<const char*>(&(transitions_[i].transition.prev_action[0])), count_action*sizeof(double));

      if (transitions_[i].transition.obs.empty())
      {
        // dymmy write of NaNs
        fileOutStream.write(reinterpret_cast<const char*>(&(dummy_obs[0])), count_obs*sizeof(double));
        fileOutStream.write(reinterpret_cast<const char*>(&(dummy_action[0])), count_action*sizeof(double));
      }
      else
      {
        fileOutStream.write(reinterpret_cast<const char*>(&(transitions_[i].transition.obs[0])), count_obs*sizeof(double));
        fileOutStream.write(reinterpret_cast<const char*>(&(transitions_[i].transition.action[0])), count_action*sizeof(double));
      }

      fileOutStream.write(reinterpret_cast<const char*>(&(transitions_[i].transition.reward)), sizeof(double));
    }
    fileOutStream.close();
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
  Predictor::update(transition);

  transitions_.push_back(CachedTransition(transition));
}

void FQIPredictor::finalize()
{
  Predictor::finalize();

  // rebuilding predictor every macro_batch_size_ episodes or do not rebuild at all
  if (macro_batch_size_ && ((++macro_batch_counter_ % macro_batch_size_) == 0))
    rebuild();
}

void FQIPredictor::rebuild()
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
    
    // Learn: first accumulate all samples and targets
    for (size_t jj=0; jj < transitions_.size(); ++jj)
    {
      CachedTransition& t = transitions_[jj];
      
      if (!t.state || reproject_state)
        t.state = projector_->project(t.transition.prev_obs, t.transition.prev_action);
      
      representation_->write(t.state, VectorConstructor(targets[jj]));
    }
    
    // second is actual learning
    representation_->finalize();
    
    CRAWL("FQI iteration " << ii << " delta L_inf: " << maxdelta);
  }
}
