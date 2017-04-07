/** \file greedy.cpp
 * \brief Greedy and Epsilon-greedy samplers source file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-01-22
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
#include <grl/samplers/greedy.h>

using namespace grl;

REGISTER_CONFIGURABLE(GreedySampler)
REGISTER_CONFIGURABLE(EpsilonGreedySampler)

void GreedySampler::request(ConfigurationRequest *config)
{
}

void GreedySampler::configure(Configuration &config)
{
  rand_ = new Rand();
}

void GreedySampler::reconfigure(const Configuration &config)
{
}

size_t GreedySampler::sample(const LargeVector &values, ActionType *at) const
{
  if (at)
    *at = atGreedy;
  
  size_t mai = 0, man = 1;
  for (size_t ii=1; ii < values.size(); ++ii)
  {
    if (values[ii] > values[mai])
    {
      mai = ii;
      man = 1;
    }
    else if (values[ii] == values[mai])
      man++;
  }
  
  if (man > 1)
  {
    // Multiple indices with same maximum value. Choose randomly.
    size_t ii = mai;
    for (int jj = rand_->getInteger(man); jj >=0; ++ii)
      if (values[ii] == values[mai])
        --jj;

    return ii-1;
  }
  else
    return mai;
}

void GreedySampler::distribution(const LargeVector &values, LargeVector *distribution) const
{
  *distribution = LargeVector::Constant(values.size(), 0.);
  (*distribution)[GreedySampler::sample(values)] = 1;
}

void EpsilonGreedySampler::request(ConfigurationRequest *config)
{
  GreedySampler::request(config);
  config->push_back(CRP("epsilon", "Exploration rate (can be defined per action)", epsilon_, CRP::Online));
  config->push_back(CRP("decay_rate", "Multiplicative decay factor per episode", decay_rate_, CRP::Configuration));
  config->push_back(CRP("decay_min", "Minimum decay (eps_min = eps*decay_min)", decay_min_, CRP::Configuration));
}

void EpsilonGreedySampler::configure(Configuration &config)
{
  GreedySampler::configure(config);
  
  epsilon_ = config["epsilon"].v();
  decay_rate_ = config["decay_rate"];
  decay_min_ = config["decay_min"];
  decay_ = 1;
  
  if (epsilon_.size() < 1)
    throw bad_param("sampler/epsilon_greedy:epsilon");
    
  if (epsilon_.size() > 1)
  {
    distribution_ = calculateBaseDistribution(epsilon_);
    distribution_sum_ = distribution_.sum();
  }
}

void EpsilonGreedySampler::reconfigure(const Configuration &config)
{
  config.get("epsilon", epsilon_);
  
  if (epsilon_.size() < 1)
    throw bad_param("sampler/epsilon_greedy:epsilon");
    
  if (epsilon_.size() > 1)
  {
    distribution_ = calculateBaseDistribution(epsilon_);
    distribution_sum_ = distribution_.sum();
  }
  
  if (config.has("action") && config["action"].str() == "reset")
    decay_ = 1;
}

size_t EpsilonGreedySampler::sample(double time, const LargeVector &values, ActionType *at)
{
  if (time == 0.)
    decay_ = fmax(decay_*decay_rate_, decay_min_);
  return sample(values, at);
}

size_t EpsilonGreedySampler::sample(const LargeVector &values, ActionType *at) const
{
  double r = rand_->get();
  
  if (epsilon_.size() > 1)
  {
    if (epsilon_.size() != values.size())
      throw bad_param("sampler/epsilon_greedy:epsilon");
      
    // Find number of eligible actions
    size_t eligible=0;
    for (size_t ii=0; ii < epsilon_.size(); ++ii)
      if (r < decay_*epsilon_[ii])
        eligible++;
        
    if (eligible > 0)
    {
      // Chose one randomly
      size_t ri = rand_->getInteger(eligible);
      for (size_t ii=0; ii < epsilon_.size(); ++ii)
        if (r < decay_*epsilon_[ii])
          if (!ri--)
          {
            if (at)
              *at = atExploratory;
            return ii;
          }
    }
  }
  else if (r < decay_*epsilon_[0])
  {
    if (at)
      *at = atExploratory;
    return rand_->getInteger(values.size());
  }

  return GreedySampler::sample(values, at);
}

void EpsilonGreedySampler::distribution(const LargeVector &values, LargeVector *distribution) const
{
  if (epsilon_.size() > 1)
  {
    *distribution = decay_*distribution_;
    (*distribution)[GreedySampler::sample(values)] += 1 - decay_*distribution_sum_;
  }
  else
  {
    GreedySampler::distribution(values, distribution);
    
    for (size_t ii=0; ii < values.size(); ++ii)
    {
      if ((*distribution)[ii] == 1)
        (*distribution)[ii] = 1-decay_*epsilon_[0];
      (*distribution)[ii] += decay_*epsilon_[0]/values.size();
    }
  }
}

LargeVector EpsilonGreedySampler::calculateBaseDistribution(const LargeVector &epsilon) const
{
  // Calculate base distribution
  LargeVector distribution = LargeVector::Constant(epsilon.size(), 0.);
  
  std::vector<double> e;
  fromVector(epsilon, e);
  e.push_back(0.); // So we can start at ii=1.
  std::sort(e.begin(), e.end());
  
  for (size_t ii=1; ii < e.size(); ++ii)
  {
    if (e[ii] != e[ii-1])
    {
      // Divvy up difference in probability among eligible actions.
      size_t eligible = 0;
      for (size_t jj=0; jj < epsilon.size(); ++jj)
        if (e[ii] <= epsilon[jj])
          eligible++;
      for (size_t jj=0; jj < epsilon.size(); ++jj)
        if (e[ii] <= epsilon[jj])
          distribution[jj] += (e[ii]-e[ii-1])/eligible;
    }
  }
  
  TRACE("Base distribution is " << distribution);
  
  return distribution;
}
