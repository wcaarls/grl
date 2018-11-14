/** \file solver.h
 * \brief Solver policy header file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2018-11-14
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

#ifndef GRL_SOLVER_POLICY_H_
#define GRL_SOLVER_POLICY_H_

#include <grl/policy.h>
#include <grl/solver.h>

namespace grl
{

/// Policy that uses a solver to calculate the action
class SolverPolicy : public Policy
{
  public:
    TYPEINFO("mapping/policy/solver", "Policy that uses a solver to calculate the action")

  protected:
    PolicySolver *solver_;
    int interval_, episodes_;

  public:
    SolverPolicy() : solver_(NULL), interval_(1), episodes_(0) { }
    
    // From Configurable  
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Policy
    virtual void act(double time, const Observation &in, Action *out);
    virtual void act(const Observation &in, Action *out) const;
};

}

#endif /* GRL_SOLVER_POLICY_H_ */
