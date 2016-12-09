/** \file rwa.cpp
 * \brief Reward weighted averaging optimizer source file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2016-12-09
 *
 * \copyright \verbatim
 * Copyright (c) 2016, Wouter Caarls
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

#include <string.h>
#include <grl/optimizers/rwa.h>

using namespace grl;	

REGISTER_CONFIGURABLE(RWAOptimizer)

void RWAOptimizer::request(ConfigurationRequest *config)
{
  config->push_back(CRP("mu", "Parent population size", mu_, CRP::Configuration, 0));
  config->push_back(CRP("lambda", "Offspring population size", lambda_, CRP::Configuration, 0));
  config->push_back(CRP("sigma", "Standard deviation of exploration", sigma_, CRP::Configuration));

  config->push_back(CRP("policy", "policy/parameterized", "Control policy prototype", policy_));
}

void RWAOptimizer::configure(Configuration &config)
{
  policy_ = (ParameterizedPolicy*)config["policy"].ptr();
  prototype_ = (ParameterizedPolicy*)policy_->clone();
  params_ = prototype_->size();
  
  mu_ = config["mu"];
  lambda_ = config["lambda"];
  sigma_ = config["sigma"].v();
  
  if (sigma_.size() == 1)
    sigma_ = LargeVector::Constant(params_, sigma_[0]);
    
  if (sigma_.size() != params_)
    throw bad_param("optimizer/rwa:sigma");
  
  if (!mu_)
    mu_ = 4+floor(3*std::log(params_));
    
  if (!lambda_)
    lambda_ = 1;
    
  reset();
}

void RWAOptimizer::reconfigure(const Configuration &config)
{
  if (config.has("action") && config["action"].str() == "reset")
  {
    LargeVector xstart = prototype_->params();
    
    INFO("Initializing (" << mu_ << ", " << lambda_ << ") reward weighted averaging optimizer with " << params_ << " parameters");
    
    population_.resize(lambda_);
    for (size_t ii=0; ii < lambda_; ++ii)
    {
      population_[ii] = Individual((ParameterizedPolicy*)prototype_->clone(), 0);
    
      for (size_t jj=0; jj < params_; ++jj)
        population_[ii].first->params()[jj] = xstart[jj] + RandGen::get()*sigma_[jj];
    }
        
    best_reward_ = 0;
    index_ = 0;
  }
}

void RWAOptimizer::report(size_t ii, double reward)
{
  population_[index_+ii].second = reward;

  if (reward > best_reward_)
  {
    INFO(population_[index_+ii].first->params() << " = " << reward);
    
    best_reward_ = reward;
    memcpy(policy_->params().data(), population_[index_+ii].first->params().data(), params_*sizeof(double));
  }
  else
    TRACE(population_[index_+ii].first->params() << " = " << reward);
    
  if (ii == lambda_-1)
  {
    index_ += lambda_;
    
    TRACE("Creating next generation (" << index_ << "-" << index_+lambda_-1 << ")");
    
    // Calculate new prototype
    LargeVector xstart = LargeVector::Constant(params_, 0.);
    double weight = 0;
    
    std::sort(population_.begin(), population_.end(), compare);
    
    for (size_t ii=0; ii < mu_ && ii < index_; ++ii)
    {
      for (size_t jj=0; jj < params_; ++jj)
        xstart[jj] += population_[index_-ii-1].first->params()[jj] * population_[index_-ii-1].second;
      weight += population_[index_-ii-1].second;
    }
    
    for (size_t jj=0; jj < params_; ++jj)
      xstart[jj] /= weight;

    CRAWL("Prototype: " << xstart);
          
    // Create next generation
    population_.resize(index_+lambda_);
    for (size_t ii=0; ii < lambda_; ++ii)
    {
      population_[index_+ii] = Individual((ParameterizedPolicy*)prototype_->clone(), 0);

      for (size_t jj=0; jj < params_; ++jj)
        population_[index_+ii].first->params()[jj] = RandGen::getNormal(xstart[jj], sigma_[jj]);
    }
  }
}
