/** \file optimizer.h
 * \brief Generic black-box optimizer defintion.
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

#ifndef GRL_OPTIMIZER_H_
#define GRL_OPTIMIZER_H_

#include <grl/configurable.h>
#include <grl/policy.h>

namespace grl
{

/// Optimizes a Policy from rollouts.
class Optimizer : public Configurable
{
  public:
    /// Returns the number of policies to evaluate.
    virtual size_t size() const = 0;
    
    /// Request a policy to evaluate.
    virtual Policy *request(size_t ii) const = 0;
    
    /**
     * \brief Report the result of a policy's evaluation.
     *
     * One all policies have been evaluated, a new set will be generated.
     * \note Not all optimizers support out-of-order reporting.
     */     
    virtual void report(size_t ii, double reward) = 0;
};

}

#endif /* GRL_OPTIMIZER_H_ */
