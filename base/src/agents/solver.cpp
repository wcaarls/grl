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
 
  config->push_back(CRP("policy", "mapping/policy", "Control policy", policy_));
  config->push_back(CRP("predictor", "predictor", "Optional (model) predictor", predictor_, true));
  config->push_back(CRP("solver", "solver", "Model-based solver", solver_));
}

void SolverAgent::configure(Configuration &config)
{
  policy_ = (Policy*)config["policy"].ptr();
  predictor_ = (Predictor*)config["predictor"].ptr();
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

TransitionType SolverAgent::start(const Vector &obs, Vector *action)
{
  if (predictor_)
    predictor_->finalize();

  time_= 0.;
  episodes_++;
  
  if (interval_ && (episodes_%interval_)==0)
    solver_->solve();
  solver_->solve(obs);

  TransitionType tt = policy_->act(time_, obs, action);
  
  prev_obs_ = obs;
  prev_action_ = *action;

  return tt;
}

TransitionType SolverAgent::step(double tau, const Vector &obs, double reward, Vector *action)
{
  time_ += tau;
  solver_->resolve(time_, obs);
  TransitionType tt = policy_->act(time_, obs, action);
  
  if (predictor_)
  {
    Transition t(prev_obs_, prev_action_, reward, obs, *action);
    predictor_->update(t);
  }

  prev_obs_ = obs;
  prev_action_ = *action;

  return tt;
}

void SolverAgent::end(double tau, const Vector &obs, double reward)
{
  if (predictor_)
  {
    Transition t(prev_obs_, prev_action_, reward, obs);
    predictor_->update(t);
  }
}

void SolverAgent::run()
{
  while (ok())
    solver_->solve();
}
