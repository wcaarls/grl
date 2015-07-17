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

#include <omp.h>
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

  config->push_back(CRP("rebuild_batch_size", "Number of episodes after which a batch is rebuild. Use 0 for no rebulds.",
                        10, CRP::Configuration, 0));
}

void FQIPredictor::configure(Configuration &config)
{
  projector_ = (Projector*)config["projector"].ptr();
  representation_ = (Representation*)config["representation"].ptr();
  policy_ = (QPolicy*)config["policy"].ptr();
  
  gamma_ = config["gamma"];
  max_samples_ = config["transitions"];
  iterations_ = config["iterations"];
  rebuild_batch_size_ = config["rebuild_batch_size"];

  // Initialize memory
  reset();
}

void FQIPredictor::reconfigure(const Configuration &config)
{
  if (config.has("action") && config["action"].str() == "reset")
  {
    DEBUG("Initializing transition store");
  
    transitions_.clear();
    rebuild_counter_ = 1;
  }
  else if (config["action"].str() == "load")
  {
    std::string file = config["file"].str() + path() + ".dat";
    std::replace(file.begin(), file.end(), '/', '_');

    std::ifstream fileInStream;
    fileInStream.open(file.c_str(),  std::ofstream::in);
    if (fileInStream.bad())
            return;
    size_t count, count_obs, count_action;
    fileInStream.read(reinterpret_cast<char*>(&count),	sizeof(size_t));
    fileInStream.read(reinterpret_cast<char*>(&count_obs),	sizeof(size_t));
    fileInStream.read(reinterpret_cast<char*>(&count_action),	sizeof(size_t));
    transitions_.reserve(count);
    Transition tr;
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
      transitions_.push_back(tr);
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
    if (fileOutStream.bad())
            return;
    size_t count = transitions_.size();
    fileOutStream.write(reinterpret_cast<const char*>(&count),	sizeof(size_t));
    size_t count_obs = transitions_[0].obs.size();
    fileOutStream.write(reinterpret_cast<const char*>(&count_obs),	sizeof(size_t));
    size_t count_action = transitions_[0].action.size();
    fileOutStream.write(reinterpret_cast<const char*>(&count_action),	sizeof(size_t));
    // prepare dymmy symbols to obay format
    Vector dummy_obs, dummy_action;
    dummy_obs.resize(count_obs, std::numeric_limits<double>::signaling_NaN());
    dummy_action.resize(count_action, std::numeric_limits<double>::signaling_NaN());
    for (unsigned int i = 0; i < count; i++)
    {
      fileOutStream.write(reinterpret_cast<const char*>(&(transitions_[i].prev_obs[0])), count_obs*sizeof(double));
      fileOutStream.write(reinterpret_cast<const char*>(&(transitions_[i].prev_action[0])), count_action*sizeof(double));

      if (transitions_[i].obs.empty())
      {
        // dymmy write
        fileOutStream.write(reinterpret_cast<const char*>(&(dummy_obs[0])), count_obs*sizeof(double));
        fileOutStream.write(reinterpret_cast<const char*>(&(dummy_action[0])), count_action*sizeof(double));
      }
      else
      {
        fileOutStream.write(reinterpret_cast<const char*>(&(transitions_[i].obs[0])), count_obs*sizeof(double));
        fileOutStream.write(reinterpret_cast<const char*>(&(transitions_[i].action[0])), count_action*sizeof(double));
      }

      fileOutStream.write(reinterpret_cast<const char*>(&(transitions_[i].reward)), sizeof(double));
    }
    fileOutStream.close();
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
{// call it macro_batch_size
  // rebuilding predictor every 'rebuild_batch_size_' episodes or do not rebuild at all
  if (rebuild_batch_size_ && ((rebuild_counter_++ % rebuild_batch_size_) == 0))
    rebuild();
}

void FQIPredictor::rebuild()
{
  std::vector<double> targets(transitions_.size(), 0.);
  double maxdelta = std::numeric_limits<double>::infinity();

  representation_->reset();

  for (size_t ii=0; ii < iterations_ && maxdelta > 0.001; ++ii)
  {
    maxdelta = 0;
  
#pragma omp parallel
{
  #pragma omp for
    // Convert to sample set
    for (size_t jj=0; jj < transitions_.size(); ++jj)
    {
      const Transition& transition = transitions_[jj];
      double target = transition.reward;
      // here we calculate current value of a value function + reward. target = reward + gamma * (value * distribution)
      if (!transition.obs.empty()) // For all transitions except terminal or absorbing
      {
        Vector values, distribution;
        policy_->values(transition.obs, &values); // values contains Q(observation,a) values approximated by LLR
        policy_->distribution(transition.obs, &distribution); // in the distribution the highest Q value action gets highest probability

        double v = 0.;
        for (size_t kk=0; kk < values.size(); ++kk)
          v += values[kk]*distribution[kk]; // values are mixed with probability, not a max function.

        target += gamma_*v;
      }

      maxdelta = fmax(maxdelta, fabs(targets[jj]-target));
      targets[jj] = target;
    }
}

    representation_->reset();

#pragma omp parallel
{
  #pragma omp for
    // Learn: first accumulate all samples and targets
    for (size_t jj=0; jj < transitions_.size(); ++jj)
    {
      const Transition& transition = transitions_[jj];
      representation_->write(projector_->project(transition.prev_obs, transition.prev_action), VectorConstructor(targets[jj]));
    }
}

    // second is actual learning
    representation_->finalize();
    
    CRAWL("FQI iteration " << ii << " delta L_inf: " << maxdelta);
  }
}
