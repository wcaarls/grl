/** \file vi.cpp
 * \brief Value iteration solver source file.
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

#include <grl/solvers/vi.h>

using namespace grl;

REGISTER_CONFIGURABLE(ValueIterationSolver)

void ValueIterationSolver::request(ConfigurationRequest *config)
{
  config->push_back(CRP("sweeps", "Number of planning sweeps before solution is returned", sweeps_, CRP::Configuration, 0));
  
  config->push_back(CRP("discretizer", "discretizer.observation", "State space discretizer", discretizer_));
  config->push_back(CRP("predictor", "predictor/full", "Predictor to iterate", predictor_));
}

void ValueIterationSolver::configure(Configuration &config)
{
  sweeps_ = config["sweeps"];
  
  discretizer_ = (Discretizer*)config["discretizer"].ptr();
  predictor_ = (Predictor*)config["predictor"].ptr();
}

void ValueIterationSolver::reconfigure(const Configuration &config)
{
}

ValueIterationSolver *ValueIterationSolver::clone() const
{
  ValueIterationSolver *solver = new ValueIterationSolver(*this);
  
  solver->discretizer_ = discretizer_->clone();
  solver->predictor_ = predictor_->clone();
  
  return solver;
}

void ValueIterationSolver::solve()
{
  for (size_t ii=0; ii < sweeps_; ++ii)
    for (Discretizer::iterator it=discretizer_->begin(); it != discretizer_->end(); ++it)
    {
      predictor_->update(Transition(*it));
      predictor_->finalize();
    }
}
