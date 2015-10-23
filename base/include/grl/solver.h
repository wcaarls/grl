/** \file solver.h
 * \brief Generic solver definition.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-08-27
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

#ifndef GRL_SOLVER_H_
#define GRL_SOLVER_H_

#include <grl/configurable.h>

namespace grl
{

/// Solves an MDP
class Solver : public Configurable
{
  public:
    virtual ~Solver() { }
    virtual Solver *clone() const = 0;
    
    /// Solve MDP.
    virtual bool solve() = 0;

    /// Solve MDP from a certain starting point.
    virtual bool solve(const Vector &x0) { }

    /// Resolve MDP based on previous solution, but from a later starting point.
    virtual bool resolve(double t, const Vector &xt) { }
};

}

#endif /* GRL_SOLVER_H_ */
