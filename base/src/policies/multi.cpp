/** \file multi.cpp
 * \brief Combining policy source file. 
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2018-03-13
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

#include <grl/policies/multi.h>

using namespace grl;

REGISTER_CONFIGURABLE(MultiPolicy)

void MultiPolicy::request(ConfigurationRequest *config)
{
  config->push_back(CRP("strategy", "Combination strategy", strategy_str_, CRP::Configuration, {"policy_strategy_add_prob", "policy_strategy_multiply_prob", "policy_strategy_majority_voting_prob", "policy_strategy_rank_voting_prob"}));
  config->push_back(CRP("discretizer", "discretizer.action", "Action discretizer", discretizer_));
  config->push_back(CRP("policy", "mapping/policy", "Sub-policies", &policy_));
}

void MultiPolicy::configure(Configuration &config)
{
  strategy_str_ = config["strategy"].str();
  if (strategy_str_ == "policy_strategy_add_prob")
    strategy_ = csAddProbabilities;
  else if (strategy_str_ == "policy_strategy_multiply_prob")
    strategy_ = csMultiplyProbabilities;
  else if (strategy_str_ == "policy_strategy_majority_voting_prob")
    strategy_ = csMajorityVotingProbabilities;
  else if (strategy_str_ == "policy_strategy_rank_voting_prob")
    strategy_ = csRankVotingProbabilities;
  else
    throw bad_param("mapping/policy/multi:strategy");

  discretizer_ = (Discretizer*)config["discretizer"].ptr();
  
  policy_ = *(ConfigurableList*)config["policy"].ptr();
  
  if (policy_.empty())
    throw bad_param("mapping/policy/multi:policy");
}

void MultiPolicy::reconfigure(const Configuration &config)
{
}

void MultiPolicy::act(const Observation &in, Action *out) const
{
  LargeVector dist;
  distribution(in, *out, &dist);
  size_t action = sample(dist);

  std::vector<Vector> variants;
  discretizer_->options(in, &variants);
  if (action >= variants.size())
  {
    ERROR("Subpolicy action out of bounds: " << action << " >= " << variants.size());
    throw bad_param("mapping/policy/multi:policy");
  }

  // rgo std::cout << "MultiPolicy::act::variants: " << variants << std::endl; //#rgo
  // rgo std::cout << "MultiPolicy::act::variants[" << action << "]: " << variants[action] << std::endl; //#rgo
  *out = variants[action];
  
  // Actually, this may be different per policy. Just to be on the safe side,
  // we always cut off all off-policy traces.
  out->type = atExploratory;
}

void MultiPolicy::act(double time, const Observation &in, Action *out)
{
  // Call downstream policies to update time
  for (size_t ii=0; ii != policy_.size(); ++ii)
  {
    Action temp = *out;
    policy_[ii]->act(time, in, &temp);
  }
    
  // Then continue with usual action selection
  act(in, out);
}

void MultiPolicy::distribution(const Observation &in, const Action &prev, LargeVector *out) const
{
  LargeVector dist;
  LargeVector param_choice;
  
  
  // rgo std::cout << "\nMultiPolicy::distribution" << " in: " << (in) << " ;prev - " << (prev) << " ;out - " << (*out) << std::endl; //#rgo
 
  switch (strategy_)
  {
    case csAddProbabilities:

      policy_[0]->distribution(in, prev, out);
      param_choice = LargeVector::Zero(out->size());
      //out = LargeVector::Zero(param_choice->size());

      for (size_t ii=0; ii != policy_.size(); ++ii)
      {
        // rgo std::cout << "MultiPolicy::distribution " << policy_.size() << " policy_[0]: " << policy_[0] <<   " policy_[ii] " << policy_[ii] << std::endl; //#rgo
        // Add subsequent policies' probabilities according to chosen strategy
        policy_[ii]->distribution(in, prev, &dist);
        if (dist.size() != out->size())
        {
          ERROR("Subpolicy " << ii << " has incompatible number of actions");
          throw bad_param("mapping/policy/multi:policy");
        }

        // rgo std::cout << "strategy_ dist.size()" << dist.size() << std::endl; //#rgo
        for (size_t jj=0; jj < dist.size(); ++jj)
        {
          // rgo std::cout << "strategy_ - jj: " << jj << " - out: " << (*out)[jj] << " - dist[jj]: " << dist[jj] << std::endl; //#rgo
          param_choice[jj] += dist[jj];
        }
      }
      
      //std::cout << "MultiPolicy::param_choice: " << param_choice << std::endl;
      other_selection(param_choice, out);
      //std::cout << "MultiPolicy::out: " << (*out) << "\n" << std::endl;
      
      break;
        
    case csMultiplyProbabilities:
        
      policy_[0]->distribution(in, prev, out);
      param_choice = LargeVector::Ones(out->size());

      for (size_t ii=0; ii != policy_.size(); ++ii)
      {
        // rgo std::cout << "MultiPolicy::distribution " << policy_.size() << " policy_[0]: " << policy_[0] <<   " policy_[ii] " << policy_[ii] << std::endl; //#rgo
        // Add subsequent policies' probabilities according to chosen strategy
        policy_[ii]->distribution(in, prev, &dist);
        if (dist.size() != out->size())
        {
          ERROR("Subpolicy " << ii << " has incompatible number of actions");
          throw bad_param("mapping/policy/multi:policy");
        }

        for (size_t jj=0; jj < dist.size(); ++jj)
        {
          param_choice[jj] *= dist[jj];
        }
      }
      
      //std::cout << "MultiPolicy::param_choice: " << param_choice << std::endl;
      other_selection(param_choice, out);
      //std::cout << "MultiPolicy::out: " << (*out) << "\n" << std::endl;
      
      break;
        
    case csMajorityVotingProbabilities:
      
      policy_[0]->distribution(in, prev, out);
      param_choice = LargeVector::Zero(out->size());
        
      for (size_t ii=0; ii != policy_.size(); ++ii)
      {
        policy_[ii]->distribution(in, prev, &dist);
        if (dist.size() != out->size())
        {
          ERROR("Subpolicy " << ii << " has incompatible number of actions");
          throw bad_param("mapping/policy/multi:policy");
        }
                
        double p_best_ind = 0.0;
        short i_ind = dist.size() - 1;
        // rgo std::cout << "\nMultiPolicy::dist: " << dist << std::endl;
        for (size_t jj=0; jj < dist.size(); ++jj)
        {
          // rgo std::cout << "MultiPolicy::prob_ind[" << jj << "]{" << prob_ind[jj]<< "} > p_best_ind{" << p_best_ind << std::endl;
          if (dist[jj] > p_best_ind)
          {
            p_best_ind = dist[jj];
            i_ind = jj;
          }
          // rgo std::cout << "MultiPolicy::p_best_ind{" << p_best_ind << "} = i_ind: " << i_ind << std::endl;
        }
        param_choice[i_ind] += 1;
      }
      
      // rgo std::cout << "MultiPolicy::param_choice: " << param_choice << std::endl;
      
      softmax(param_choice, out);
      
      // rgo std::cout << "MultiPolicy::out: " << (*out) << std::endl;
      
      break;
        
    case csRankVotingProbabilities:

      break;
  }      
}

void MultiPolicy::softmax(const LargeVector &values, LargeVector *distribution) const
{
  LargeVector v = LargeVector::Zero(values.size());
  for (size_t ii=0; ii < values.size(); ++ii)
    if (std::isnan(values[ii]))
      ERROR("SoftmaxSampler: NaN value in Boltzmann distribution 1");

  distribution->resize(values.size());
  const double threshold = -100;

  // Find max_power and min_power, and center of feasible powers
  double max_power = -DBL_MAX;
  for (size_t ii=0; ii < values.size(); ++ii)
  {
    double p = values[ii]/tau_;
    max_power = (max_power < p) ? p : max_power;
  }
  double min_power = max_power + threshold;
  double center = (max_power+min_power)/2.0;

  // Discard powers from interval [0.0; threshold] * max_power
  double sum = 0;
  for (size_t ii=0; ii < values.size(); ++ii)
  {
    double p = values[ii]/tau_;
    if (p > min_power)
    {
      p -= center;
      v[ii] = exp(p);
      sum += v[ii];
      (*distribution)[ii] = 1;

      if (std::isnan(v[ii]))
        ERROR("SoftmaxSampler: NaN value in Boltzmann distribution 2");
    }
    else
      (*distribution)[ii] = 0;
  }

  for (size_t ii=0; ii < values.size(); ++ii)
  {
    (*distribution)[ii] *= v[ii]/sum;
    if (std::isnan((*distribution)[ii]))
      ERROR("SoftmaxSampler: NaN value in Boltzmann distribution 4");
  }
}

void MultiPolicy::other_selection(const LargeVector &values, LargeVector *distribution) const
{
  
  
  LargeVector v = LargeVector::Zero(values.size());
  for (size_t ii=0; ii < values.size(); ++ii)
    if (std::isnan(values[ii]))
      ERROR("OtherSelectionMultiPolicy: NaN value in  distribution 1");

  distribution->resize(values.size());
  const double threshold = -100;

  // Find max_power and min_power, and center of feasible powers
  double max_power = -DBL_MAX;
  for (size_t ii=0; ii < values.size(); ++ii)
  {
    double p = 1.0/tau_;
    max_power = (max_power < p) ? p : max_power;
  }
  double min_power = max_power + threshold;
  double center = (max_power+min_power)/2.0;

  // Discard powers from interval [0.0; threshold] * max_power
  double sum = 0;
  for (size_t ii=0; ii < values.size(); ++ii)
  {
    double p = 1.0/tau_;
    if (p > min_power)
    {
      p -= center;
      v[ii] = pow(values[ii], p);
      sum += v[ii];
      (*distribution)[ii] = 1;

      if (std::isnan(v[ii]))
        ERROR("OtherSelectionMultiPolicy: NaN value in  distribution 2");
    }
    else
      (*distribution)[ii] = 0;
  }

  for (size_t ii=0; ii < values.size(); ++ii)
  {
    (*distribution)[ii] *= v[ii]/sum;
    if (std::isnan((*distribution)[ii]))
      ERROR("OtherSelectionMultiPolicy: NaN value in  distribution 4");
  }
}