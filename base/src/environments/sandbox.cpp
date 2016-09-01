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
#include <iomanip>
#include <../../addons/rbdl/include/grl/environments/rbdl_leo.h> // remove if direction is set in task

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

  cum_falls_ = 0;
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
  sandbox_->start(ConstantVector(1, test), &state_);
  task_->observe(state_, obs, &terminal);

  obs_ = *obs;
  state_obj_->set(state_);

  test_ = test;

  if (exporter_)
    exporter_->open((test_?"test":"learn"), (test_?time_test_:time_learn_) != 0.0);

  prev_time_test_ = time_test_;
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

  if ((!test_) && (*terminal == 2))
    cum_falls_++;

  state_ = next;
  obs_ = *obs;
  state_obj_->set(state_);

  return tau;
}

void SandboxEnvironment::report(std::ostream &os) const
{
  const int pw = 15;
  std::stringstream progressString;
  progressString << std::fixed << std::setprecision(3) << std::right;
  progressString << std::setw(pw) << cum_falls_;
  progressString << std::setw(pw) << (time_test_ - prev_time_test_);
  os << progressString.str();

  sandbox_->report(os);
  task_->report(os);
}

//-------------------------------------------------------------
void SandboxDynamicalModel::request(ConfigurationRequest *config)
{
  dm_.request(config);
  config->push_back(CRP("dof_count", "int.dof_count", "Number of degrees of freedom of the model", dof_count_, CRP::Configuration, 0, INT_MAX));
  config->push_back(CRP("target_env", "environment", "Interaction environment", target_env_, true));
}

void SandboxDynamicalModel::configure(Configuration &config)
{
  dm_.configure(config);
  dof_count_ = config["dof_count"];
  target_env_ = (Environment*)config["target_env"].ptr(); // here we can select a real enviromnent if needed
}

SandboxDynamicalModel *SandboxDynamicalModel::clone() const
{
  SandboxDynamicalModel *model = new SandboxDynamicalModel();
  model->dm_ = dm_;
  return model;
}

void SandboxDynamicalModel::start(const Vector &hint, Vector *state)
{
  if (target_env_)
  {
    // reduce state
    Vector state0;
    state0.resize(2*dof_count_);
    state0 << state->block(0, 0, 1, 2*dof_count_);

    target_env_->start(0, &state0);

    *state << state0,
              0.0,  // rlsTime
              0.28, // rlsRefRootHeight, possible values 0.28 and 0.35
              ConstantVector(rlsStateDim - 2*rlsDofDim - 2, 0); // initialize the rest to zero
  }

//  std::cout << *state << std::endl;

  state_ = *state;
}

double SandboxDynamicalModel::step(const Vector &action, Vector *next)
{
  // reduce state
  Vector state0;
  state0.resize(2*dof_count_+1);
  state0 << state_.block(0, 0, 1, 2*dof_count_+1);

//  std::cout << state0 << std::endl;
//  std::cout << action << std::endl;

  // auto-actuate arm
  Vector action0;
  action0.resize(dof_count_);
  if (action.size() == 3)
  {
    double armVoltage = (14.0/3.3) * 5.0*(-0.26 - state_[rlsArmAngle]);
    action0 << action, armVoltage;
    std::cout << "  > Action: " << action0 << std::endl;
  }
  else
    action0 << action;

  //action0 << ConstantVector(4, 0);

  // call dynamics of the reduced state
  Vector next0;
  next0.resize(state0.size());
  double tau;
  if (target_env_)
  {
    tau = target_env_->step(action0, &next0, NULL, NULL);
    next0[rlsTime] = state0[rlsTime] + tau;
  }
  else
    tau = dm_.step(state0, action0, &next0);

//  std::cout << "GRL: " << next0 << std::endl;
/*
  // augment state
  double t = next0[2*dof_count_];
  int direction = state_[2*dof_count_+1] > 0.30;
  int tt = (int)round(t/tau);
  int t5 = (int)round(5/tau);
  if ( tt % t5 == 0 ) // change setpoint every 5 seconds
    direction = 1 - direction;
  next->resize(2*dof_count_+2);
  *next << next0.block(0, 0, 1, 2*dof_count_+1), (direction?0.35:0.28);
  dm_.dynamics_->finalize(*next);
*/

  next->resize(2*dof_count_+2);
  *next << next0, state_[2*dof_count_+1]; // fake direction

  dm_.dynamics_->finalize(*next);

  if ( fabs((*next)[rlsComVelocityZ] - 0.0) < 0.01)
  {
    if ( fabs((*next)[rlsRootZ] - 0.28) < 0.01)
      (*next)[rlsRefRootZ] = 0.35;
    else if ( fabs((*next)[rlsRootZ] - 0.35) < 0.01)
      (*next)[rlsRefRootZ] = 0.28;
  }


//  (*next)[rlsRefRootZ] = 0.35;

  std::cout << "  > Height: " << (*next)[rlsRootZ] << std::endl;

  std::cout << "  > Next state: " << *next << std::endl;

  export_meshup_animation(action0, *next);

  state_ = *next;
  return tau;
}

double SandboxDynamicalModel::export_meshup_animation(const Vector &action, const Vector &next) const
{
  std::ofstream data_stream;
  data_stream.open ("sd_leo.csv", std::ios_base::trunc);
  if (!data_stream || 2*dof_count_ > next.size())
  {
    std::cerr << "Error opening file sd_leo.csv" << std::endl;
    abort();
  }
  data_stream << next[rlsTime] << ", ";
  for (int i = 0; i < 2*dof_count_-1; i++)
    data_stream << next[i] << ", ";
  data_stream << next[2*dof_count_-1] << std::endl;
  data_stream.close();

  data_stream.open ("u_leo.csv", std::ios_base::trunc);
  if (!data_stream || dof_count_ > action.size())
  {
    std::cerr << "Error opening file u_leo.csv" << std::endl;
    abort();
  }
  data_stream << next[rlsTime] << ", ";
  for (int i = 0; i < dof_count_-1; i++)
    data_stream << action[i] << ", ";
  data_stream << action[dof_count_-1] << std::endl;
  data_stream.close();
}

