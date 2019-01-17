/** \file integrator.cpp
 * \brief Integrator environment source file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2018-05-18
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

#include <grl/environments/integrator.h>

using namespace grl;

REGISTER_CONFIGURABLE(IntegratorDynamics)
REGISTER_CONFIGURABLE(IntegratorRegulatorTask)

void IntegratorDynamics::request(ConfigurationRequest *config)
{
  config->push_back(CRP("order", "int.order", "Order of integrator (number of state dimensions)", (int)order_, CRP::Configuration));
}

void IntegratorDynamics::configure(Configuration &config)
{
  order_ = config["order"];
}

void IntegratorDynamics::reconfigure(const Configuration &config)
{
}

void IntegratorDynamics::eom(const Vector &state, const Vector &actuation, Vector *xd) const
{
  if (state.size() != order_+1 || actuation.size() != 1)
    throw Exception("dynamics/integrator requires a task/integrator subclass");

  xd->resize(order_+1);
  for (size_t ii=0; ii < order_-1; ++ii)
    (*xd)[ii] = state[ii+1];
  (*xd)[order_-1] = actuation[0];  
  (*xd)[order_] = 1;
}

// Regulator

void IntegratorRegulatorTask::request(ConfigurationRequest *config)
{
  config->push_back(CRP("order", "int.order", "Order of integrator (number of state dimensions)", (int)order_, CRP::System));
  RegulatorTask::request(config);
}

void IntegratorRegulatorTask::configure(Configuration &config)
{
  RegulatorTask::configure(config);
  
  order_ = config["order"];
  
  if (q_.size() != order_)
    throw bad_param("task/integrator/regulator:q");
  if (r_.size() != 1)
    throw bad_param("task/integrator/regulator:r");
    
  Vector omax = ConstantVector(order_, 1), omin = -omax;

  config.set("observation_min", omin);
  config.set("observation_max", omax);
  config.set("action_min", VectorConstructor(-1));
  config.set("action_max", VectorConstructor(1));
}

void IntegratorRegulatorTask::reconfigure(const Configuration &config)
{
  RegulatorTask::reconfigure(config);
}

void IntegratorRegulatorTask::observe(const Vector &state, Observation *obs, int *terminal) const
{
  if (state.size() != order_+1)
    throw Exception("task/integrator/regulator requires dynamics/integrator");
    
  obs->v.resize(order_);
  for (size_t ii=0; ii < order_; ++ii)
    (*obs)[ii] = state[ii];
  obs->absorbing = false;

  *terminal = state[2] > 3;
}

bool IntegratorRegulatorTask::invert(const Observation &obs, Vector *state, double time) const
{
  *state = extend(obs, VectorConstructor(time));
  
  return true;
}
