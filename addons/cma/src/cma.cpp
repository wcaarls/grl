/** \file cma.cpp
 * \brief CMA-ES optimizer source file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-02-13
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

#include <string.h>
#include <cma/cmaes_interface.h>
#include <grl/optimizers/cma.h>

using namespace grl;	

REGISTER_CONFIGURABLE(CMAOptimizer)

void CMAOptimizer::request(ConfigurationRequest *config)
{
  config->push_back(CRP("population", "Population size", population_, CRP::Configuration, 0));
  config->push_back(CRP("sigma", "Initial standard deviation (a single-element vector will be replicated for all parameters)", sigma_, CRP::Configuration));

  config->push_back(CRP("policy", "policy/parameterized", "Control policy prototype", policy_));
}

void CMAOptimizer::configure(Configuration &config)
{
  policy_ = (ParameterizedPolicy*)config["policy"].ptr();
  prototype_ = policy_->clone();
  params_ = prototype_->size();
  
  population_ = config["population"];
  sigma_ = config["sigma"];
  
  if (sigma_.size() == 1)
    sigma_ = ConstantVector(params_, sigma_[0]);
    
  if (sigma_.size() != params_)
    throw bad_param("optimizer/cma:sigma");
  
  if (!population_)
    population_ = 4+floor(3*std::log(params_));
  
  policies_.resize(population_);
  fitness_.resize(population_);
  
  for (size_t ii=0; ii < population_; ++ii)
    policies_[ii] = prototype_->clone();
    
  reset();
}

void CMAOptimizer::reconfigure(const Configuration &config)
{
  if (config.has("action") && config["action"].str() == "reset")
  {
    Vector xstart = prototype_->params(), stddev;
    
    INFO("Initializing CMA-ES optimizer with population size " << population_ << " (" << params_ << " parameters)");
  
    bzero(&evo_, sizeof(evo_));
    cmaes_init(&evo_, params_, xstart.data(), sigma_.data(), lrand48(), population_, "non");
    
    // Sample initial population
    double *const *pop = cmaes_SamplePopulation(&evo_);
    
    for (size_t ii=0; ii < population_; ++ii)
      memcpy(policies_[ii]->params().data(), pop[ii], params_*sizeof(double));
      
    best_reward_ = -std::numeric_limits<double>::infinity();
  }
}

CMAOptimizer *CMAOptimizer::clone() const
{
  return NULL;
}

void CMAOptimizer::report(size_t ii, double reward)
{
  if (reward > best_reward_)
  {
    INFO(policies_[ii]->params() << " = " << reward);
    
    best_reward_ = reward;
    memcpy(policy_->params().data(), policies_[ii]->params().data(), params_*sizeof(double));
  }
  else
    DEBUG(policies_[ii]->params() << " = " << reward);

  // cmaes minimizes fitness
  fitness_[ii] = -reward;
  
  if (ii == population_-1)
  {
    INFO("Updating search distribution");
  
    // Update search distribution  
    cmaes_UpdateDistribution(&evo_, fitness_.data());
    
    const char *s = cmaes_TestForTermination(&evo_);
    if (s) NOTICE(s);
    
    // Sample next population
    double *const *pop = cmaes_SamplePopulation(&evo_);

    for (size_t ii=0; ii < population_; ++ii)
      memcpy(policies_[ii]->params().data(), pop[ii], params_*sizeof(double));
  }
}
