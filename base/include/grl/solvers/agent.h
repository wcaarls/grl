/** \file agent.h
 * \brief Agent solver header file.
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

#ifndef GRL_AGENT_SOLVER_H_
#define GRL_AGENT_SOLVER_H_

#include <grl/solver.h>
#include <grl/environments/observation.h>
#include <grl/agent.h>

namespace grl
{

/// Solver that uses a simulated agent internally
class AgentSolver : public Solver
{
  public:
    TYPEINFO("solver/agent", "Solver that uses a simulated agent");

  protected:
    ObservationModel *model_;
    Agent *agent_;
    VectorSignal *state_;
    size_t steps_, horizon_;
    Vector start_;
    
  public:
    AgentSolver() : model_(NULL), agent_(NULL), state_(NULL), steps_(100), horizon_(100) { }
  
    // From Configurable    
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Solver
    virtual bool solve();
};

}

#endif /* GRL_AGENT_SOLVER_H_ */
