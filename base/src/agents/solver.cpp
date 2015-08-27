/** \file solver.cpp
 * \brief Solver agent source file.
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

#include <grl/agents/solver.h>

using namespace grl;

REGISTER_CONFIGURABLE(SolverAgent)

void SolverAgent::request(ConfigurationRequest *config)
{
  config->push_back(CRP("interval", "Episodes between successive solutions (0=asynchronous)", interval_, CRP::Configuration, 1, INT_MAX));
 
  config->push_back(CRP("policy", "policy", "Control policy", policy_));
  config->push_back(CRP("predictor", "predictor/model", "Model predictor", predictor_, true));
  config->push_back(CRP("solver", "solver", "Model-based solver", solver_));
}

void SolverAgent::configure(Configuration &config)
{
  policy_ = (Policy*)config["policy"].ptr();
  predictor_ = (ModelPredictor*)config["predictor"].ptr();
  solver_ = (Solver*)config["solver"].ptr();
  interval_ = config["interval"];
  episodes_ = 0;
  
  if (!interval_)
    start();
}

void SolverAgent::reconfigure(const Configuration &config)
{
  episodes_ = 0;
}

SolverAgent *SolverAgent::clone() const
{
  SolverAgent *agent = new SolverAgent(*this);
  
  agent->policy_ = policy_->clone();
  agent->predictor_= predictor_->clone();
  agent->solver_= solver_->clone();
  
  return agent;
}

void SolverAgent::start(const Vector &obs, Vector *action)
{
  time_= 0.;
  episodes_++;
  policy_->act(time_, obs, action);
  
  if (interval_ && (episodes_%interval_)==0)
    solver_->solve();
  
  prev_obs_ = obs;
  prev_action_ = *action;
}

void SolverAgent::step(double tau, const Vector &obs, double reward, Vector *action)
{
  time_ += tau;
  policy_->act(time_, obs, action);
  
  if (predictor_)
  {
    Transition t(prev_obs_, prev_action_, reward, obs, *action);
    predictor_->update(t);
  }

  prev_obs_ = obs;
  prev_action_ = *action;  
}

void SolverAgent::end(double tau, double reward)
{
  if (predictor_)
  {
    Transition t(prev_obs_, prev_action_, reward);
    predictor_->update(t);
  }
}

void SolverAgent::run()
{
  while (ok())
    solver_->solve();
}
