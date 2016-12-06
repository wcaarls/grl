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

GreedySampler *GreedySampler::clone()
{
  GreedySampler *gs = new GreedySampler(*this);
  gs->rand_ = rand_->clone();

  return gs;
}

size_t GreedySampler::sample(const LargeVector &values) const
{
  size_t mai = 0;

  for (size_t ii=1; ii < values.size(); ++ii)
    if (values[ii] > values[mai])
      mai = ii;
/*
 * Commented code selects random index in case of multiple maximumum values
 *
  LargeVector same_values = LargeVector::Constant(values.size(), 0);
  size_t jj = 0;
  for (size_t ii=0; ii < values.size(); ++ii)
    if (values[ii] == values[mai])
      same_values[jj++] = ii;

  if (jj != 0)
    mai = same_values[rand_->getInteger(jj)];
*/
  return mai;
}

void GreedySampler::distribution(const LargeVector &values, LargeVector *distribution) const
{
  *distribution = LargeVector::Constant(values.size(), 0.);
  (*distribution)[GreedySampler::sample(values)] = 1;
}

void EpsilonGreedySampler::request(ConfigurationRequest *config)
{
  config->push_back(CRP("epsilon", "Exploration rate (can be defined per action)", epsilon_, CRP::Online));
}

void EpsilonGreedySampler::configure(Configuration &config)
{
  GreedySampler::configure(config);
  
  epsilon_ = config["epsilon"].v();
  
  if (epsilon_.size() < 1)
    throw bad_param("sampler/epsilon_greedy:epsilon");
    
  if (epsilon_.size() > 1)
  {
    distribution_ = calculateBaseDistribution(epsilon_);
    distribution_sum_ = sum(distribution_);
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
    distribution_sum_ = sum(distribution_);
  }
}

EpsilonGreedySampler *EpsilonGreedySampler::clone()
{
  EpsilonGreedySampler *egs = new EpsilonGreedySampler(*this);
  egs->rand_ = rand_->clone();
  
  return egs;
}

size_t EpsilonGreedySampler::sample(const LargeVector &values) const
{
  double r = rand_->get();
  
  if (epsilon_.size() > 1)
  {
    if (epsilon_.size() != values.size())
      throw bad_param("sampler/epsilon_greedy:epsilon");
      
    // Find number of eligible actions
    size_t eligible=0;
    for (size_t ii=0; ii < epsilon_.size(); ++ii)
      if (r < epsilon_[ii])
        eligible++;
        
    if (eligible > 0)
    {
      // Chose one randomly
      size_t ri = rand_->getInteger(eligible);
      for (size_t ii=0; ii < epsilon_.size(); ++ii)
        if (r < epsilon_[ii])
          if (!ri--)
            return ii;
    }
  }
  else
    if (r < epsilon_[0])
      return rand_->getInteger(values.size());

  return GreedySampler::sample(values);
}

void EpsilonGreedySampler::distribution(const LargeVector &values, LargeVector *distribution) const
{
  if (epsilon_.size() > 1)
  {
    *distribution = distribution_;
    (*distribution)[GreedySampler::sample(values)] += 1 - distribution_sum_;
  }
  else
  {
    GreedySampler::distribution(values, distribution);
    
    for (size_t ii=0; ii < values.size(); ++ii)
    {
      if ((*distribution)[ii] == 1)
        (*distribution)[ii] = 1-epsilon_[0];
      (*distribution)[ii] += epsilon_[0]/values.size();
    }
  }
}

Vector EpsilonGreedySampler::calculateBaseDistribution(const Vector &epsilon) const
{
  // Calculate base distribution
  Vector distribution = Vector::Constant(epsilon.size(), 0.);
  
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
        if (e[ii] < epsilon[jj])
          eligible++;
      for (size_t jj=0; jj < epsilon.size(); ++jj)
        if (e[ii] < epsilon[jj])
          distribution[jj] += (e[ii]-e[ii-1])/eligible;
    }
  }
  
  return distribution;
}
