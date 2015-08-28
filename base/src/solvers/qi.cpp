/** \file qi.cpp
 * \brief Q iteration solver source file.
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

#include <iomanip>

#include <grl/solvers/qi.h>

using namespace grl;

REGISTER_CONFIGURABLE(QIterationSolver)

void QIterationSolver::request(ConfigurationRequest *config)
{
  config->push_back(CRP("sweeps", "Number of planning sweeps before solution is returned", sweeps_, CRP::Configuration, 0));
  config->push_back(CRP("gamma", "Discount rate", gamma_));
  
  config->push_back(CRP("model", "observation_model", "Observation model used for planning", model_));

  config->push_back(CRP("state_discretizer", "discretizer.observation", "State space discretizer", state_discretizer_));
  config->push_back(CRP("action_discretizer", "discretizer.action", "Action discretizer", action_discretizer_));
  config->push_back(CRP("projector", "projector.pair", "Projects observations onto representation space", projector_));
  config->push_back(CRP("representation", "representation.value/action", "Value function representation", representation_));
}

void QIterationSolver::configure(Configuration &config)
{
  sweeps_ = config["sweeps"];
  gamma_ = config["gamma"];
  
  model_ = (ObservationModel*)config["model"].ptr();
  state_discretizer_ = (Discretizer*)config["state_discretizer"].ptr();
  action_discretizer_ = (Discretizer*)config["action_discretizer"].ptr();
  action_discretizer_->options(&variants_);

  projector_ = (Projector*)config["projector"].ptr();
  representation_ = (Representation*)config["representation"].ptr();
}

void QIterationSolver::reconfigure(const Configuration &config)
{
}

QIterationSolver *QIterationSolver::clone() const
{
  QIterationSolver *solver = new QIterationSolver(*this);
  
  solver->model_ = model_->clone();
  solver->state_discretizer_ = state_discretizer_->clone();
  solver->action_discretizer_ = action_discretizer_->clone();
  solver->projector_ = projector_->clone();
  solver->representation_ = representation_->clone();
  
  return solver;
}

void QIterationSolver::solve()
{
  for (Discretizer::iterator it=state_discretizer_->begin(); it != state_discretizer_->end(); ++it)
  {
    for (size_t aa=0; aa < variants_.size(); ++aa)
    {
      Vector next;
      double reward;
      int terminal;
      model_->step(*it, variants_[aa], &next, &reward, &terminal);
      
      if (!next.empty())
      {
        if (!terminal)
        {
          // Find value of best action
          Vector value;
          double v=-std::numeric_limits<double>::infinity();
          for (size_t ii=0; ii < variants_.size(); ++ii)
            v = fmax(v, representation_->read(projector_->project(next, variants_[ii]), &value));
          
          reward += gamma_*v;
        }
      
        representation_->write(projector_->project(*it, variants_[aa]), VectorConstructor(reward));
      }
    }
  }
}
