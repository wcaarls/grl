/** \file solver.cpp
 * \brief Solver policy source file.
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

#include <grl/policies/solver.h>

using namespace grl;

REGISTER_CONFIGURABLE(SolverPolicy)

void SolverPolicy::request(ConfigurationRequest *config)
{
  config->push_back(CRP("interval", "Episodes between successive solutions (0=asynchronous)", interval_, CRP::Configuration, 1, INT_MAX));
  config->push_back(CRP("solver", "solver/policy", "Solver that calculates the policy", solver_));
}

void SolverPolicy::configure(Configuration &config)
{
  solver_ = (PolicySolver*)config["solver"].ptr();
  interval_ = config["interval"];
  
  episodes_ = 0;
}

void SolverPolicy::reconfigure(const Configuration &config)
{
  if (config.has("action") && config["action"].str() == "reset")
    episodes_ = 0; 
}

void SolverPolicy::act(double time, const Observation &in, Action *out)
{
  if (time == 0.)
  {
    episodes_++;
    if (interval_ && (episodes_%interval_)==0)
      solver_->solve();
    solver_->solve(in);
  }
  else
    solver_->resolve(time, in);
    
  return solver_->policy()->act(in, out);
}

void SolverPolicy::act(const Observation &in, Action *out) const
{
  return solver_->policy()->act(in, out);
}
