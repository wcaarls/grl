/** \file cma_optimizer.h
 * \brief CMA-ES optimizer header file.
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

#ifndef CMA_OPTIMIZER_H_
#define CMA_OPTIMIZER_H_

#include <cma/cmaes.h>
#include <grl/optimizer.h>
#include <grl/policies/parameterized.h>

namespace grl
{

/// Coverance matrix adaptation optimizer.
class CMAOptimizer : public Optimizer
{
  public:
    TYPEINFO("optimizer/cma", "Coverance matrix adaptation black-box optimizer")
 
  protected:
    ParameterizedPolicy *prototype_, *policy_;
    std::vector<ParameterizedPolicy*> policies_;
    Vector sigma_;
    
    cmaes_t evo_;
    size_t population_, params_;
    Vector fitness_;
    double best_reward_;
  
  public:
    CMAOptimizer() : prototype_(NULL), policy_(NULL), sigma_(VectorConstructor(1.)), population_(0), params_(0), best_reward_(0) { }

    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);
    
    // From Optimizer  
    virtual CMAOptimizer *clone() const;
    virtual size_t size() const { return population_; }
    virtual Policy *request(size_t ii) const { return policies_[ii]; }
    virtual void report(size_t ii, double reward);
};

}

#endif /* CMA_OPTIMIZER_H_ */
