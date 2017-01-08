/** \file rbdl_leo_model.cpp
 * \brief RBDL file for C++ description of Leo model.
 *
 * \author    Ivan Koryakovskiy <i.koryakovskiy@tudelft.nl>
 * \date      2016-09-13
 *
 * \copyright \verbatim
 * Copyright (c) 2016, Ivan Koryakovskiy
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

#include <grl/environments/leo/rbdl_leo_model.h>
#include <grl/environments/leo/rbdl_leo_task.h>

using namespace grl;

REGISTER_CONFIGURABLE(LeoSquattingSandboxModel)

void LeoSandboxModel::request(ConfigurationRequest *config)
{
  dm_.request(config);

  config->push_back(CRP("target_dof", "int.target_dof", "Number of degrees of freedom of the target model", target_dof_, CRP::Configuration, 0, INT_MAX));
  config->push_back(CRP("animation", "Save current state or full animation", animation_, CRP::Configuration, {"nope", "full", "immediate"}));
  config->push_back(CRP("target_env", "environment", "Interaction environment", target_env_, true));
}

void LeoSandboxModel::configure(Configuration &config)
{
  dm_.configure(config);

  target_env_ = (Environment*)config["target_env"].ptr(); // Select a real enviromnent if needed
  target_dof_ = config["target_dof"];
  animation_ = config["animation"].str();
}

void LeoSandboxModel::export_meshup_animation(const Vector &state, const Vector &action) const
{
  if (animation_ == "nope")
    return;

  std::ios_base::openmode om = (animation_ == "full" && action.size() != 0)?(std::ios_base::app):(std::ios_base::trunc);

  std::ofstream data_stream;
  data_stream.open("sd_leo.csv", om);
  if (!data_stream || 2*target_dof_ > state.size())
  {
    std::cerr << "Error opening file sd_leo.csv" << std::endl;
    abort();
  }
  data_stream << state[rlsTime] << ", ";
  for (int i = 0; i < 2*target_dof_-1; i++)
    data_stream << state[i] << ", ";
  data_stream << state[2*target_dof_-1] << std::endl;
  data_stream.close();

  data_stream.open ("u_leo.csv", om);
  if (!data_stream || target_dof_ > action.size())
  {
    std::cerr << "Error opening file u_leo.csv" << std::endl;
    abort();
  }
  data_stream << state[rlsTime] << ", ";
  for (int i = 0; i < target_dof_-1; i++)
    data_stream << action[i] << ", ";
  data_stream << action[target_dof_-1] << std::endl;
  data_stream.close();
}

///////////////////////////////////////////////

void LeoSquattingSandboxModel::request(ConfigurationRequest *config)
{
  LeoSandboxModel::request(config);

  config->push_back(CRP("lower_height", "double.lower_height", "Lower bound of root height to switch direction", lower_height_, CRP::Configuration, 0.0, DBL_MAX));
  config->push_back(CRP("upper_height", "double.upper_height", "Upper bound of root height to switch direction", upper_height_, CRP::Configuration, 0.0, DBL_MAX));
}

void LeoSquattingSandboxModel::configure(Configuration &config)
{
  LeoSandboxModel::configure(config);

  lower_height_ = config["lower_height"];
  upper_height_ = config["upper_height"];
}

void LeoSquattingSandboxModel::start(const Vector &hint, Vector *state)
{
  // Obtain an actual state from a target environment, not from task
  if (target_env_)
    target_env_->start(0, state);

  // Unknown bug in GRL call/Lua/RBDL: need to call eom, then 'finalize' works correctly
  Vector xd;
  action_step_.resize(target_dof_);
  dm_.dynamics_->eom(*state, action_step_, &xd);

  // Fill parts of a state such as Center of Mass, Angular Momentum
  dm_.dynamics_->finalize(*state, rbdl_addition_);

  // Compose a complete state <state, time, height, com, ..., squats>
  state_.resize(stsStateDim);
  state_ << *state, VectorConstructor(lower_height_), rbdl_addition_, VectorConstructor(0.0);
  *state = state_;

  export_meshup_animation(state_, ConstantVector(target_dof_, 0));

  TRACE("Initial state: " << state_);
}

double LeoSquattingSandboxModel::step(const Vector &action, Vector *next)
{
  state_step_.resize(2*target_dof_+1);
  next_step_.resize(2*target_dof_+1);
  action_step_.resize(target_dof_);
  next->resize(state_.size());

  // reduce state
  state_step_ << state_.block(0, 0, 1, 2*target_dof_+1);

  // auto-actuate arm
  if (action.size() == target_dof_-1)
  {
    double armVoltage = (14.0/3.3) * 5.0*(-0.26 - state_[rlsArmAngle]);
    armVoltage = fmin(10.7, fmax(armVoltage, -10.7)); // ensure voltage within limits
    action_step_ << action, armVoltage;
  }
  else
    action_step_ << action;

//  action_step_ << ConstantVector(target_dof_, 0);

//  std::cout << state_step_ << std::endl;
//  std::cout << "  > Action: " << action_step_ << std::endl;

  // call dynamics of the reduced state
  double tau;
  if (target_env_)
  {
    tau = target_env_->step(action_step_, &next_step_, NULL, NULL);
    next_step_[rlsTime] = state_step_[rlsTime] + tau;
  }
  else
    tau = dm_.step(state_step_, action_step_, &next_step_);

  dm_.dynamics_->finalize(next_step_, rbdl_addition_);

  // Compose the next state
  (*next) << next_step_, VectorConstructor(state_[rlsRefRootZ]),
      rbdl_addition_, VectorConstructor(state_[stsSquats]);

  // Switch setpoint if needed
  if ( fabs((*next)[rlsComVelocityZ] - 0.0) < 0.01)
  {
    if ( fabs((*next)[rlsRootZ] - lower_height_) < 0.01)
      (*next)[rlsRefRootZ] = upper_height_;
    else if ( fabs((*next)[rlsRootZ] - upper_height_) < 0.01)
      (*next)[rlsRefRootZ] = lower_height_;
  }

  // Increase number of squats if needed
  if ((*next)[rlsRefRootZ] != state_[rlsRefRootZ])
    (*next)[stsSquats] = state_[stsSquats] + 1;

//  std::cout << "  > Height: " << (*next)[rlsRootZ] << std::endl;
//  std::cout << "  > Next state: " << *next << std::endl;

  export_meshup_animation(*next, action_step_);

  state_ = *next;
  return tau;
}
