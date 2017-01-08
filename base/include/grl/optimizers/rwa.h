/** \file rwa_optimizer.h
 * \brief Reward weighted averaging optimizer header file.
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

#ifndef RWA_OPTIMIZER_H_
#define RWA_OPTIMIZER_H_

#include <grl/optimizer.h>
#include <grl/policies/parameterized.h>

namespace grl
{

/// Kormushev-style reward weighted averaging optimizer.
class RWAOptimizer : public Optimizer
{
  public:
    TYPEINFO("optimizer/rwa", "Reward weighted averaging black-box optimizer")
    
    typedef std::pair<ParameterizedPolicy*,double> Individual;
 
  protected:
    ParameterizedPolicy *policy_, *prototype_;
    size_t mu_, lambda_;
    LargeVector sigma_;
    
    std::vector<Individual> population_;
    double best_reward_;
    size_t index_, params_;
    
  public:
    RWAOptimizer() : policy_(NULL), prototype_(NULL), mu_(0), lambda_(1), sigma_(VectorConstructor(1.)), best_reward_(0.), index_(0) { }

    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);
    
    // From Optimizer  
    virtual size_t size() const { return lambda_; }
    virtual Policy *request(size_t ii) const { return population_[index_+ii].first; }
    virtual void report(size_t ii, double reward);
    
  protected:
    static bool compare(const Individual &a, const Individual &b)
    {
      return a.second < b.second;
    }
};

}

#endif /* RWA_OPTIMIZER_H_ */
