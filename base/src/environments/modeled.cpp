/** \file modeled.cpp
 * \brief Modeled environment and dynamical model source file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-01-22
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

#include <grl/environment.h>

using namespace grl;

REGISTER_CONFIGURABLE(ModeledEnvironment)
REGISTER_CONFIGURABLE(DynamicalModel)

void ModeledEnvironment::request(ConfigurationRequest *config)
{
  config->push_back(CRP("model", "model", "Environment model", model_));
  config->push_back(CRP("task", "task", "Task to perform in the environment (should match model)", task_));

  config->push_back(CRP("state", "state", "Current state of the model", CRP::Provided));
}

void ModeledEnvironment::configure(Configuration &config)
{
  model_ = (Model*)config["model"].ptr();
  task_ = (Task*)config["task"].ptr();
  
  state_obj_ = new State();
  
  config.set("state", state_obj_);
}

void ModeledEnvironment::reconfigure(const Configuration &config)
{
}
    
ModeledEnvironment *ModeledEnvironment::clone() const
{
  ModeledEnvironment* me = new ModeledEnvironment();
  
  me->model_ = model_;
  me->task_ = task_;
  
  return me;
}

void ModeledEnvironment::start(Vector *obs)
{
  int terminal;

  task_->start(&state_);
  task_->observe(state_, obs, &terminal);
  
  state_obj_->set(state_);
}

void ModeledEnvironment::step(const Vector &action, Vector *obs, double *reward, int *terminal)
{
  Vector next;

  model_->step(state_, action, &next);
  task_->observe(next, obs, terminal);
  task_->evaluate(state_, action, next, reward);
  state_ = next;
  
  state_obj_->set(state_);
}

void DynamicalModel::request(ConfigurationRequest *config)
{
  config->push_back(CRP("control_step", "double.control_step", "Control step time", tau_, CRP::Configuration, 0.001, DBL_MAX));
  config->push_back(CRP("integration_steps", "Number of integration steps per control step", (int)steps_, CRP::Configuration, 1));

  config->push_back(CRP("dynamics", "dynamics", "Equations of motion", dynamics_));
}

void DynamicalModel::configure(Configuration &config)
{
  dynamics_ = (Dynamics*)config["dynamics"].ptr();
  
  tau_ = config["control_step"];
  steps_ = config["integration_steps"];
}

void DynamicalModel::reconfigure(const Configuration &config)
{
}

DynamicalModel *DynamicalModel::clone() const
{
  DynamicalModel *dm = new DynamicalModel();
  dm->dynamics_ = dynamics_;
  return dm;
}

void DynamicalModel::step(const Vector &state, const Vector &action, Vector *next) const
{
  Vector xd;
  double h = tau_/steps_;
  
  *next = state;
  
  for (size_t ii=0; ii < steps_; ++ii)
  {
    dynamics_->eom(*next, action, &xd);
    Vector k1 = h*xd;
    dynamics_->eom(*next + k1/2, action, &xd);
    Vector k2 = h*xd;
    dynamics_->eom(*next + k2/2, action, &xd);
    Vector k3 = h*xd;
    dynamics_->eom(*next + k3, action, &xd);
    Vector k4 = h*xd;

    *next = *next + (k1+2*k2+2*k3+k4)/6;
  }
}
