/** \file vi.h
 * \brief Value iteration solver header file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-08-28
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

#ifndef GRL_VALUE_ITERATION_SOLVER_H_
#define GRL_VALUE_ITERATION_SOLVER_H_

#include <grl/solver.h>
#include <grl/discretizer.h>
#include <grl/environments/observation.h>
#include <grl/projector.h>
#include <grl/representation.h>

namespace grl
{

/// Solve MDPs by value iteration.
class ValueIterationSolver : public Solver
{
  public:
    TYPEINFO("solver/vi", "Value iteration solver");

  protected:
    Discretizer *state_discretizer_, *action_discretizer_;
    ObservationModel *model_;
    Projector *projector_;
    Representation *representation_;
    
    size_t sweeps_;
    double gamma_;
    
    std::vector<Vector> variants_;
    
  public:
    ValueIterationSolver() : state_discretizer_(NULL), action_discretizer_(NULL), model_(NULL), projector_(NULL), representation_(NULL), sweeps_(1), gamma_(0.97) { }
  
    // From Configurable    
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Solver
    virtual ValueIterationSolver *clone() const;
    virtual void solve();
};

}

#endif /* GRL_VALUE_ITERATION_SOLVER_H_ */
