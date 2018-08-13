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

REGISTER_CONFIGURABLE(DiscreteMultiPolicy)

REGISTER_CONFIGURABLE(MultiPolicy)

void MultiPolicy::request(ConfigurationRequest *config)
{
  config->push_back(CRP("strategy", "Combination strategy", strategy_str_, CRP::Configuration, {"policy_strategy_binning_prob", "policy_strategy_density_based_prob", "policy_strategy_data_center_prob"}));
  config->push_back(CRP("policy", "mapping/policy", "Sub-policies", &policy_));

  config->push_back(CRP("bins", "Binning Simple Discretization", bins_));
  config->push_back(CRP("r_distance_parameter", "R Distance Parameter", r_distance_parameter_));
  
  config->push_back(CRP("output_min", "vector.action_min", "Lower limit on outputs", min_, CRP::System));
  config->push_back(CRP("output_max", "vector.action_max", "Upper limit on outputs", max_, CRP::System));
}

void MultiPolicy::configure(Configuration &config)
{
  strategy_str_ = config["strategy"].str();
  if (strategy_str_ == "policy_strategy_binning_prob")
    strategy_ = csBinningProbabilities;
  else if (strategy_str_ == "policy_strategy_density_based_prob")
    strategy_ = csDensityBasedProbabilities;
  else if (strategy_str_ == "policy_strategy_data_center_prob")
    strategy_ = csDataCenterProbabilities;
  else
    throw bad_param("mapping/policy/multi:strategy");

  bins_ = config["bins"];
  r_distance_parameter_ = config["r_distance_parameter"];

  min_ = config["output_min"].v();
  max_ = config["output_max"].v();

  if (min_.size() != max_.size() || !min_.size())
    throw bad_param("policy/action:{output_min,output_max}");

  policy_ = *(ConfigurableList*)config["policy"].ptr();
  
  if (policy_.empty())
    throw bad_param("mapping/policy/multi:policy is empty");
  
}

void MultiPolicy::reconfigure(const Configuration &config)
{
}

void MultiPolicy::act(const Observation &in, Action *out) const
{
  LargeVector dist;
 
  Action tmp_action;
  double bin_result = 0.0;
  double range = (max_[0] - min_[0]);
  double step = range/bins_;
  
  double bin_tmp = 0;
  double tmp = 0.0;
  
  double mean_bin = 0;
  double int_mean_bin = 0;
  double choose_bin = 0;
  double normalized = 0.0;
  
  double tmp_final_policy = 0;

  switch (strategy_)
  {
    case csBinningProbabilities:
      // CRAWL("MultiPolicy::policy_.size(): " << policy_.size());
      // CRAWL("MultiPolicy::range: " << range);
      // CRAWL("MultiPolicy::step: " << step);

      for (size_t ii=0; ii != policy_.size(); ++ii)
      {
        policy_[ii]->act(in, &tmp_action);
        bin_tmp = 0;
        tmp = (double) tmp_action.v[0];

        //3 (10) 2.4 (9) 1.8 (8) 1.2 (7) 0.6 (6) 0
        //   (5) -0.6 (4) -1.2 (3) -1.8 (2) -2.4 (1) -3

        while (tmp > min_[0])
        {
          bin_tmp++; 
          tmp = tmp - step;
        }
        bin_result += bin_tmp;
        
        // CRAWL("MultiPolicy::policy_[ii=" << ii << "]->act(in, &tmp_action): " << tmp_action);
        // CRAWL("MultiPolicy::bin_tmp: " << bin_tmp);
        // CRAWL("MultiPolicy::bin_result: " << bin_result);
      }

      mean_bin = bin_result/policy_.size();
      // CRAWL("MultiPolicy::mean_bin: " << mean_bin << "\n");
      int_mean_bin = (int) mean_bin;
      // CRAWL("MultiPolicy::int_mean_bin: " << int_mean_bin << "\n");
      choose_bin = int_mean_bin;
      if ( fabs(int_mean_bin + 1 - mean_bin) > 0.5 ) 
        choose_bin++;
      // CRAWL("MultiPolicy::choose_bin: " << choose_bin << "\n");
      tmp_final_policy = (--choose_bin*step + 0.5*step) + min_[0];
      // CRAWL("MultiPolicy::tmp_final_policy: " << tmp_final_policy << "\n");
      tmp_action.v = tmp_final_policy;
      dist = ConstantLargeVector(1, tmp_final_policy);
      // CRAWL("MultiPolicy::tmp_action.v: " << tmp_action.v << "\n");
      // CRAWL("MultiPolicy::rgo_debug");
      break;
        
    case csDensityBasedProbabilities:
    {
      dist = LargeVector::Zero(policy_.size());
      //LargeVector density = LargeVector::Zero(policy_.size());
      
      for (size_t ii=0; ii != policy_.size(); ++ii)
      {
        policy_[ii]->act(in, &tmp_action);
        normalized = (((double) tmp_action.v[0])-min_[0])/(max_[0] - min_[0]);
        dist[ii] = normalized;
        CRAWL("MultiPolicy::policy_[ii=" << ii << "]->act(in, &tmp_action): " << tmp_action);
        CRAWL("MultiPolicy::normalized: " << normalized);
      }
      
      double* density = new double[policy_.size()];
      for(size_t ii=0; ii != policy_.size(); ++ii)
      {
        double tmp = 0;
        for(size_t jj=0; jj != policy_.size(); ++jj)
        {
          CRAWL("MultiPolicy::(dist[ii] - dist[jj]): " << fabs(dist[ii] - dist[jj]));
          CRAWL("MultiPolicy::sqrt((dist[ii] - dist[jj])): " << sqrt((dist[ii] - dist[jj])));
          if ((dist[ii] - dist[jj]) != 0)
          {
            CRAWL("MultiPolicy::tmp: " << tmp);
            tmp = tmp + sqrt( fabs(dist[ii] - dist[jj]) );
            CRAWL("MultiPolicy::tmp: " << tmp);
          }
        }
        density[ii] = -(tmp/pow(r_distance_parameter_,2));//exp(-(tmp/pow(r_distance_parameter_,2)));
        CRAWL("MultiPolicy::density[" << ii << "]: " << density[ii]);
      }

      int min_indice = 0;
      for(size_t ii=1; ii < policy_.size(); ++ii)
      {
        if(density[ii] > density[ii-1])
        {
          min_indice = ii;
        }
      }
      CRAWL("MultiPolicy::min_indice: " << min_indice);
      
      policy_[min_indice]->act(in, &tmp_action);
      dist = ConstantLargeVector(1, tmp_action.v[0]);
    }
      break;
        
    case csDataCenterProbabilities:
      throw bad_param("mapping/policy/multi:policy/csDataCenterProbabilities");
      break;
  }

  *out = dist;//variants[action];
  
  CRAWL("MultiPolicy::(*out): " << (*out) << "\n");
  // Actually, this may be different per policy. Just to be on the safe side,
  // we always cut off all off-policy traces.
  out->type = atExploratory;
  
}

void DiscreteMultiPolicy::request(ConfigurationRequest *config)
{
  config->push_back(CRP("strategy", "Combination strategy", strategy_str_, CRP::Configuration, {"policy_strategy_add_prob", "policy_strategy_multiply_prob", "policy_strategy_majority_voting_prob", "policy_strategy_rank_voting_prob"}));
  config->push_back(CRP("tau", "Temperature of Boltzmann distribution", tau_));
  config->push_back(CRP("discretizer", "discretizer.action", "Action discretizer", discretizer_));
  config->push_back(CRP("policy", "mapping/policy/discrete", "Sub-policies", &policy_));
}

void DiscreteMultiPolicy::configure(Configuration &config)
{
  CRAWL("tmp in DiscreteMultiPolicy::configure(Configuration &config)");
  
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
    throw bad_param("mapping/policy/discrete/multi:strategy");

  tau_ = config["tau"];
  
  discretizer_ = (Discretizer*)config["discretizer"].ptr();
  
  policy_ = *(ConfigurableList*)config["policy"].ptr();
  
  if (policy_.empty())
    throw bad_param("mapping/policy/discrete/multi:policy");
  
  CRAWL("tmp out DiscreteMultiPolicy::configure(Configuration &config)");
}

void DiscreteMultiPolicy::reconfigure(const Configuration &config)
{
}

void DiscreteMultiPolicy::act(const Observation &in, Action *out) const
{
  LargeVector dist;
  distribution(in, *out, &dist);
  size_t action = sample(dist);

  std::vector<Vector> variants;
  discretizer_->options(in, &variants);
  if (action >= variants.size())
  {
    ERROR("Subpolicy action out of bounds: " << action << " >= " << variants.size());
    throw bad_param("mapping/policy/discrete/multi:policy");
  }

  *out = variants[action];
  
  // Actually, this may be different per policy. Just to be on the safe side,
  // we always cut off all off-policy traces.
  out->type = atExploratory;
}

void DiscreteMultiPolicy::act(double time, const Observation &in, Action *out)
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

void DiscreteMultiPolicy::distribution(const Observation &in, const Action &prev, LargeVector *out) const
{
  LargeVector dist;
  LargeVector param_choice;
  
  switch (strategy_)
  {
    case csAddProbabilities:

      policy_[0]->distribution(in, prev, out);
      param_choice = LargeVector::Zero(out->size());
      
      for (size_t ii = 0; ii != out->size(); ++ii) {
        if (std::isnan((*out)[ii]))
        {
          ERROR("MultiPolicy::param_choice::csAddProbabilities policy_[0] out(ii:" << ii << ") " << (*out)[ii]);
          for (size_t kk=0; kk < out->size(); ++kk)
            ERROR("MultiPolicy::dist::csAddProbabilities out(kk:" << kk << ") " << (*out)[kk]);
        }
      }
      
      for (size_t ii=0; ii != policy_.size(); ++ii)
      {
        // Add subsequent policies' probabilities according to chosen strategy
        policy_[ii]->distribution(in, prev, &dist);
        
        CRAWL("MultiPolicy::dist: " << dist);
        
        if (dist.size() != out->size())
        {
          ERROR("Subpolicy " << ii << " has incompatible number of actions");
          throw bad_param("mapping/policy/discrete/multi:policy");
        }

        for (size_t jj=0; jj < dist.size(); ++jj)
        {
          param_choice[jj] += dist[jj];
        
          if (std::isnan(param_choice[jj]))
          {
            ERROR("MultiPolicy::param_choice::csAddProbabilities (jj:" << jj << ") " << param_choice[jj]);
            for (size_t kk=0; kk < dist.size(); ++kk)
              ERROR("MultiPolicy::dist::csAddProbabilities (kk:" << kk << ") " << dist[kk]);
          }
        }
      }
      
      CRAWL("DiscreteMultiPolicy::param_choice: " << param_choice);

      normalized_function(param_choice, out);
      CRAWL("DiscreteMultiPolicy::out: " << (*out) << "\n");
      break;
        
    case csMultiplyProbabilities:
        
      policy_[0]->distribution(in, prev, out);
      param_choice = LargeVector::Ones(out->size());

      for (size_t ii=0; ii != policy_.size(); ++ii)
      {
        // Multiply subsequent policies' probabilities according to chosen strategy
        policy_[ii]->distribution(in, prev, &dist);
        
        CRAWL("MultiPolicy::dist: " << dist);
        
        if (dist.size() != out->size())
        {
          ERROR("Subpolicy " << ii << " has incompatible number of actions");
          throw bad_param("mapping/policy/discrete/multi:policy");
        }

        for (size_t jj=0; jj < dist.size(); ++jj)
          param_choice[jj] *= dist[jj];
      }
      
      CRAWL("DiscreteMultiPolicy::param_choice: " << param_choice);
      normalized_function(param_choice, out);
      CRAWL("DiscreteMultiPolicy::out: " << (*out) << "\n");
      
      break;
        
    case csMajorityVotingProbabilities:
      
      policy_[0]->distribution(in, prev, out);
      param_choice = LargeVector::Zero(out->size());
        
      for (size_t ii=0; ii != policy_.size(); ++ii)
      {
        policy_[ii]->distribution(in, prev, &dist);
        
        CRAWL("MultiPolicy::dist: " << dist);
        
        if (dist.size() != out->size())
        {
          ERROR("Subpolicy " << ii << " has incompatible number of actions");
          throw bad_param("mapping/policy/discrete/multi:policy");
        }
                
        double p_best_ind = 0.0;
        short i_ind = dist.size() - 1;
        for (size_t jj=0; jj < dist.size(); ++jj)
        {
          if (dist[jj] > p_best_ind)
          {
            p_best_ind = dist[jj];
            i_ind = jj;
          }
        }
        param_choice[i_ind] += 1;
      }
      
      CRAWL("DiscreteMultiPolicy::param_choice: " << param_choice);
      softmax(param_choice, out);
      CRAWL("DiscreteMultiPolicy::out: " << (*out));
      
      break;
        
    case csRankVotingProbabilities:
      {
      
      LargeVector rank_weights;
      
      policy_[0]->distribution(in, prev, out);
      param_choice = LargeVector::Zero(out->size());
        
      for (size_t ii=0; ii != policy_.size(); ++ii)
      {
        policy_[ii]->distribution(in, prev, &dist);
        
        CRAWL("MultiPolicy::dist: " << dist);
        
        if (dist.size() != out->size())
        {
          ERROR("Subpolicy " << ii << " has incompatible number of actions");
          throw bad_param("mapping/policy/discrete/multi:policy");
        }
        
        CRAWL("DiscreteMultiPolicy::dist: " << dist);
        //raking        
        rank_weights = LargeVector::Ones(out->size());
        for (size_t jj=0; jj < dist.size(); ++jj)
        {
          
          for (size_t kk = 0; kk < dist.size(); kk++)
          {
            if(dist[jj] > dist[kk])
              ++rank_weights[jj];
            CRAWL("DiscreteMultiPolicy::dist[: " << jj << "](" << dist[jj] << ") > dist [" << kk << "](" << dist[kk] << "): rank_weights[" << jj << "]("  << rank_weights[jj] << ")");
          }
          CRAWL("DiscreteMultiPolicy::policy: " << ii);
          CRAWL("DiscreteMultiPolicy::rank_weights: " << rank_weights);
          CRAWL("\nDiscreteMultiPolicy::param_choice: " << param_choice << '\n');
          param_choice[jj] += rank_weights[jj];
          CRAWL("\nDiscreteMultiPolicy::param_choice: " << param_choice << '\n');
        }
      }
      
      CRAWL("\nDiscreteMultiPolicy::param_choice: " << param_choice << '\n');
      softmax(param_choice, out);
      CRAWL("DiscreteMultiPolicy::out: " << (*out));
      }
      break;
  }      
}

void DiscreteMultiPolicy::softmax(const LargeVector &values, LargeVector *distribution) const
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

void DiscreteMultiPolicy::normalized_function(const LargeVector &values, LargeVector *distribution) const
{  
  LargeVector v = LargeVector::Zero(values.size());
  for (size_t ii=0; ii < values.size(); ++ii)
    if (std::isnan(values[ii]))
      ERROR("normalized_function: NaN value in  distribution 1");

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
        ERROR("normalized_function: NaN value in  distribution 2");
    }
    else {
      (*distribution)[ii] = 0;
    }
  }

  for (size_t ii=0; ii < values.size(); ++ii)
  {
    (*distribution)[ii] *= v[ii]/sum;
    if (std::isnan((*distribution)[ii]))
      ERROR("normalized_function: NaN value in  distribution 4");
  }
}
