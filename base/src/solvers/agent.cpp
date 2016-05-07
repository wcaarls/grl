/** \file agent.cpp
 * \brief Agent solver source file.
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

#include <iomanip>

#include <grl/solvers/agent.h>

using namespace grl;

REGISTER_CONFIGURABLE(AgentSolver)

void AgentSolver::request(ConfigurationRequest *config)
{
  config->push_back(CRP("steps", "Number of planning steps before solution is returned", steps_, CRP::Configuration, 0));
  config->push_back(CRP("horizon", "Planning episode length", horizon_, CRP::Configuration, 0));
  config->push_back(CRP("start", "Starting state for planning", start_));

  config->push_back(CRP("model", "observation_model", "Observation model used for planning", model_));
  config->push_back(CRP("agent", "agent", "Agent used for planning episodes", agent_));
  
  config->push_back(CRP("state", "state", "Current observed state of planning", CRP::Provided));
}

void AgentSolver::configure(Configuration &config)
{
  steps_ = config["steps"];
  horizon_ = config["horizon"];
  start_ = config["start"].v();
  
  model_ = (ObservationModel*)config["model"].ptr();
  agent_ = (Agent*)config["agent"].ptr();
  
  state_ = new State();
  
  config.set("state", state_);
}

void AgentSolver::reconfigure(const Configuration &config)
{
}

AgentSolver *AgentSolver::clone() const
{
  AgentSolver *solver = new AgentSolver(*this);
  
  solver->model_ = model_->clone();
  solver->agent_= agent_->clone();
  solver->state_ = new State();
  
  return solver;
}

bool AgentSolver::solve()
{
  Vector obs, action;
  int terminal=1;
  size_t steps=0;

  for (size_t ii=0; ii < steps_; ++ii)
  {
    if (terminal)
    {
      steps = 0;
      obs = start_;
      state_->set(obs);
      agent_->start(obs, &action);
    }
      
    Vector next;
    double reward;
  
    double tau = model_->step(obs, action, &next, &reward, &terminal);
    
    obs = next;
    state_->set(obs);
        
    // Guard against failed model prediction    
    if (obs.size())
    {
      if (terminal == 2)
        agent_->end(tau, obs, reward);
      else
        agent_->step(tau, obs, reward, &action);
    }
    else
      terminal = 1;
      
    // Break episodes after a while
    if (horizon_ && steps++ == horizon_)
      terminal = 1;
  }
  
  return true;
}
