/** \file solver.h
 * \brief Solver agent header file.
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

#ifndef GRL_SOLVER_AGENT_H_
#define GRL_SOLVER_AGENT_H_

#include <itc/itc.h>

#include <grl/agent.h>
#include <grl/policy.h>
#include <grl/predictors/model.h>
#include <grl/solver.h>

namespace grl
{

/// Solver model-based learning agent.
class SolverAgent : public Agent, public itc::Thread
{
  public:
    TYPEINFO("agent/solver", "Agent that successively solves learned models of the environment")

  protected:
    Policy *policy_;
    Predictor *predictor_;
    Solver *solver_;
    
    Observation prev_obs_;
    Action prev_action_;
    int interval_, episodes_;
    
    double time_;
    
  public:
    SolverAgent() : policy_(NULL), predictor_(NULL), solver_(NULL), interval_(1), episodes_(0), time_(0.) { }
  
    // From itc::Thread
    using itc::Thread::start;

    // From Configurable    
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Agent
    virtual void start(const Observation &obs, Action *action);
    virtual void step(double tau, const Observation &obs, double reward, Action *action);
    virtual void end(double tau, const Observation &obs, double reward);
    
  protected:
    virtual void run();
};

}

#endif /* GRL_SOLVER_AGENT_H_ */
