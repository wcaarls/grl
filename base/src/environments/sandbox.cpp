/** \file sandbox.cpp
 * \brief Non-Markov environment and dynamical model source file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2016-01-22
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

REGISTER_CONFIGURABLE(SandboxEnvironment)
REGISTER_CONFIGURABLE(SandboxDynamicalModel)

void SandboxEnvironment::request(ConfigurationRequest *config)
{
  config->push_back(CRP("model", "sandbox_model", "Environment model", sandbox_));
  config->push_back(CRP("task", "task", "Task to perform in the environment (should match model)", task_));
  config->push_back(CRP("exporter", "exporter", "Optional exporter for transition log (supports time, state, observation, action, reward, terminal)", exporter_, true));
  config->push_back(CRP("state", "state", "Current state of the model", CRP::Provided));
}

void SandboxEnvironment::configure(Configuration &config)
{
  sandbox_ = (Sandbox*)config["model"].ptr();
  task_ = (Task*)config["task"].ptr();
  exporter_ = (Exporter*)config["exporter"].ptr();

  if (exporter_)
  {
    // Register headers
    exporter_->init({"time", "state", "observation", "action", "reward", "terminal"});
  }

  state_obj_ = new State();

  config.set("state", state_obj_);
}

void SandboxEnvironment::reconfigure(const Configuration &config)
{
  if (config.has("action") && config["action"].str() == "reset")
    time_learn_ = time_test_ = 0.;
}

SandboxEnvironment *SandboxEnvironment::clone() const
{
  SandboxEnvironment* me = new SandboxEnvironment();

  me->sandbox_ = sandbox_;
  me->task_ = task_;

  return me;
}

void SandboxEnvironment::start(int test, Vector *obs)
{
  int terminal;

  task_->start(test, &state_);
  task_->observe(state_, obs, &terminal);
  sandbox_->start(ConstantVector(1, test), &state_);

  obs_ = *obs;
  state_obj_->set(state_);

  test_ = test;

  if (exporter_)
    exporter_->open((test_?"test":"learn"), (test_?time_test_:time_learn_) != 0.0);
}

double SandboxEnvironment::step(const Vector &action, Vector *obs, double *reward, int *terminal)
{
  Vector next;

  double tau = sandbox_->step(action, &next);
  task_->observe(next, obs, terminal);
  task_->evaluate(state_, action, next, reward);

  double &time = test_?time_test_:time_learn_;

  if (exporter_)
    exporter_->write({VectorConstructor(time), state_, *obs, action, VectorConstructor(*reward), VectorConstructor((double)*terminal)});

  time += tau;

  state_ = next;
  obs_ = *obs;
  state_obj_->set(state_);

  return tau;
}

//-------------------------------------------------------------
void SandboxDynamicalModel::request(ConfigurationRequest *config)
{
  dm_.request(config);
  config->push_back(CRP("dof_count", "int.dof_count", "Number of degrees of freedom of the model", dof_count_, CRP::Configuration, 0, INT_MAX));
}

void SandboxDynamicalModel::configure(Configuration &config)
{
  dm_.configure(config);
  dof_count_ = config["dof_count"];
}

SandboxDynamicalModel *SandboxDynamicalModel::clone() const
{
//  SandboxDynamicalModel *dm = new SandboxDynamicalModel();
//  dm->dynamics_ = dynamics_;
//  return dm;
}

void SandboxDynamicalModel::start(const Vector &hint, Vector *state)
{
//  if (exporter_)
//    exporter_->open((test_?"test":"learn"), time_ != 0.0);
  state_ = *state;
}

double SandboxDynamicalModel::step(const Vector &action, Vector *next)
{
  // reduce state
  Vector state0;
  state0.resize(2*dof_count_ + 1);
  state0 << state_.block(0, 0, 1, 2*dof_count_), state_[state_.size()-1];

  std::cout << state0 << std::endl;
//  std::cout << action << std::endl;

  // call dynamics of the reduced state
  Vector next0;
  next0.resize(state0.size());
  double tau = dm_.step(state0, action, &next0);

  std::cout << "GRL: " << next0 << std::endl;

  // augment state
  dm_.dynamics_->finalize(*next);
  next->resize(state_.size());
  double t = next0[next0.size()-1];
  int sitted_setpoint = state_[state_.size()-2];
  int tt = (int)round(t/tau);
  int t5 = (int)round(5/tau);
  if ( tt % t5 == 0 ) // change setpoint every 5 seconds
    sitted_setpoint = 1 - sitted_setpoint;
  *next << next0.block(0, 0, 1, 2*dof_count_), sitted_setpoint, t;

  std::cout << "GRL: " << *next << std::endl;

  state_ = *next;
  return tau;
}
