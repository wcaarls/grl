/** \file rbdl_leo.cpp
 * \brief RBDL file for C++ description of Leo task.
 *
 * \author    Ivan Koryakovskiy <i.koryakovskiy@tudelft.nl>
 * \date      2016-06-30
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

#include <sys/stat.h>
#include <libgen.h>
#include <iomanip>

#include <rbdl/rbdl.h>
#include <rbdl/addons/luamodel/luamodel.h>

#include <grl/lua_utils.h>
#include <grl/environments/rbdl_leo.h>
//#include <grl/environments/leohelper.h>

using namespace grl;

REGISTER_CONFIGURABLE(LeoRBDLDynamics)
REGISTER_CONFIGURABLE(LeoSquatTask)
REGISTER_CONFIGURABLE(LeoSquatTaskFA)


/*
void LeoRBDLDynamics::finalize(Vector &state)
{
  RBDLState *rbdl = rbdl_state_.instance();

  size_t dim = rbdl->model->dof_count;

  if (state.size() != 2*dim+1)
    throw Exception("dynamics/rbdl_leo is incompatible with specified task");

  leo_helper_.model = rbdl->model;
  leo_helper_.calcMomentum();

  RigidBodyDynamics::Math::Vector3d com = leo_helper_.calcCenterOfMass();

  Vector augmented_state;
  augmented_state.resize(state.size()+2);
  augmented_state << state, com[0], com[2];
  state = augmented_state;

}
*/

//-----------------------------------------------------------

void LeoSquatTask::request(ConfigurationRequest *config)
{
  Task::request(config);
  config->push_back(CRP("timeout", "double.timeout", "Task timeout", timeout_, CRP::System, 0.0, DBL_MAX));
}

void LeoSquatTask::configure(Configuration &config)
{
  action_dims_ = 4;
  timeout_ = config["timeout"];

  config.set("observation_dims", 2*rlsDofDim + 1); // 2*dof + time
  std::vector<double> obs_min = {-M_PI, -M_PI, -M_PI, -M_PI, -10*M_PI, -10*M_PI, -10*M_PI, -10*M_PI, 0};
  std::vector<double> obs_max = { M_PI,  M_PI,  M_PI,  M_PI,  10*M_PI,  10*M_PI,  10*M_PI,  10*M_PI, 1};
  toVector(obs_min, true_obs_min_);
  toVector(obs_max, true_obs_max_);
  config.set("observation_min", true_obs_min_);
  config.set("observation_max", true_obs_max_);
  config.set("action_dims", action_dims_);
  config.set("action_min", VectorConstructor(-10.7, -10.7, -10.7, -10.7));
  config.set("action_max", VectorConstructor( 10.7,  10.7,  10.7,  10.7));
  config.set("reward_min", VectorConstructor(-1000));
  config.set("reward_max", VectorConstructor( 1000));
}

void LeoSquatTask::reconfigure(const Configuration &config)
{
}
 
LeoSquatTask *LeoSquatTask::clone() const
{
  return new LeoSquatTask(*this);
}
 
void LeoSquatTask::start(int test, Vector *state) const
{
  *state = ConstantVector(rlsStateDim, 0);
  *state <<
         1.0586571916803691E+00,
        -2.1266836153365212E+00,
         1.0680264236561250E+00,
        -2.5999999999984957E-01,
        -0.0,
        -0.0,
        -0.0,
        -0.0,  // end of rlsDofDim
         0.0,  // rlsTime
         0.28, // rlsRefRootHeight, possible values 0.28 and 0.35
         ConstantVector(rlsStateDim - 2*rlsDofDim - 2, 0); // initialize the rest to zero

  (*state)[rlsRootZ] = 0.28;

  root_height_ = 0;
  squats_ = 0;
}

int LeoSquatTask::failed(const Vector &state) const
{
  double torsoAngle = state[rlsAnkleAngle] + state[rlsKneeAngle] + state[rlsHipAngle];
  if ((torsoAngle < -1.0) || (torsoAngle > 1.0) ||
      // penalty for high joint velocities
      (state[rlsAnkleAngleRate] < true_obs_min_[rlsAnkleAngleRate]) ||
      (state[rlsAnkleAngleRate] > true_obs_max_[rlsAnkleAngleRate]) ||
      (state[rlsKneeAngleRate]  < true_obs_min_[rlsKneeAngleRate])  ||
      (state[rlsKneeAngleRate]  > true_obs_max_[rlsKneeAngleRate])  ||
      (state[rlsHipAngleRate]   < true_obs_min_[rlsHipAngleRate])   ||
      (state[rlsHipAngleRate]   > true_obs_max_[rlsHipAngleRate])   ||
      (state[rlsArmAngleRate]   < true_obs_min_[rlsArmAngleRate])   ||
      (state[rlsArmAngleRate]   > true_obs_max_[rlsArmAngleRate])   ||
      // lower-upper leg colision result in large velocities
      (std::isnan(state[rlsRootZ]))
      )
    return 1;
  else
  {
    return 0;
  }
}

void LeoSquatTask::observe(const Vector &state, Vector *obs, int *terminal) const
{
  grl_assert(state.size() == rlsStateDim);

  bool reduced = false;

  if (!reduced)
  {
    obs->resize(2*rlsDofDim + 1);
   *obs << state.block(0, 0, 1, 2*rlsDofDim), state[rlsRefRootZ];
  }
  else
  {
    std::cout << state << std::endl;

    std::cout << state.block(0, 0, 1, 3) << std::endl; // angle
    std::cout << state.block(0, 4, 1, 3) << std::endl; // angle rate
    std::cout << state[8] << std::endl; // direction indicator

    obs->resize(state.size()-3);
    (*obs) << state.block(0, 0, 1, 3), state.block(0, 4, 1, 3), state[8];
  }

  if (state[rlsTime] >= timeout_)
    *terminal = 1;
  else if (failed(state))
    *terminal = 2;
  else
    *terminal = 0;
}

void LeoSquatTask::evaluate(const Vector &state, const Vector &action, const Vector &next, double *reward) const
{
  grl_assert(next.size() == rlsStateDim);

  if (failed(next))
  {
    *reward = -10000;
    return;
  }

  *reward = 0;

  // calculate support center from feet positions
  double suppport_center = 0.5 * (next[rlsLeftTipX] + next[rlsLeftHeelX]);

  // track: || root_z - h_ref ||_2^2
  *reward +=  pow(100.0 * (next[rlsRootZ] - next[rlsRefRootZ]), 2);

  // track: || com_x,y - support center_x,y ||_2^2
  *reward +=  pow( 10.00 * (next[rlsComX] - suppport_center), 2);

  *reward +=  pow( 10.00 * next[rlsComVelocityX], 2);
  *reward +=  pow( 10.00 * next[rlsComVelocityZ], 2);

  *reward +=  pow( 10.00 * next[rlsAngularMomentumY], 2);

  // NOTE: sum of lower body angles is equal to angle between ground slope
  //       and torso. Minimizing deviation from zero keeps torso upright
  //       during motion execution.
  *reward += pow(10.00 * ( next[rlsAnkleAngle] + next[rlsKneeAngle] + next[rlsHipAngle] - (0.15) ), 2); // desired torso angle

  // regularize: || q - q_desired ||_2^2
  *reward += pow(1.0 * (next[rlsArmAngle] - (-0.26)), 2); // arm

  // shaping
  double shaping = pow(30.0 * next[rlsRootZ] - state[rlsRootZ], 2);
  int s = (next[rlsRootZ] > state[rlsRootZ]) ? 1 : -1;
  s *= (next[rlsRefRootZ] > next[rlsRootZ]) ? 1 : -1;
  shaping *= s;
  *reward += -shaping;

  // negate
  *reward = - *reward;

  // for progress report
  root_height_ = next[rlsRootZ];
  if (next[rlsRefRootZ] != state[rlsRefRootZ])
    squats_++;
}
 
bool LeoSquatTask::invert(const Vector &obs, Vector *state) const
{
  return true;
}

Matrix LeoSquatTask::rewardHessian(const Vector &state, const Vector &action) const
{
  Matrix hessian;
  return hessian;
}

void LeoSquatTask::report(std::ostream &os) const
{
  const int pw = 15;
  std::stringstream progressString;
  progressString << std::fixed << std::setprecision(3) << std::right;
  progressString << std::setw(pw) << root_height_;
  progressString << std::setw(pw) << squats_;
  os << progressString.str();
}

/////////////////////////////////////////////////

void LeoSquatTaskFA::configure(Configuration &config)
{
  action_dims_ = 3;
  timeout_ = config["timeout"];

  // True observations: 2*dof + time
  std::vector<double> obs_min = {-M_PI, -M_PI, -M_PI, -M_PI, -10*M_PI, -10*M_PI, -10*M_PI, -10*M_PI, 0};
  std::vector<double> obs_max = { M_PI,  M_PI,  M_PI,  M_PI,  10*M_PI,  10*M_PI,  10*M_PI,  10*M_PI, 1};
  toVector(obs_min, true_obs_min_);
  toVector(obs_max, true_obs_max_);

  // Observations and actions exposed to an agent
  int agent_obs_dim = 2*(rlsDofDim-1) + 1;
  config.set("observation_dims", agent_obs_dim);
  Vector observation_min, observation_max;
  observation_min.resize(agent_obs_dim);
  observation_min << true_obs_min_[rlsAnkleAngle], true_obs_min_[rlsKneeAngle], true_obs_min_[rlsHipAngle],
      true_obs_min_[rlsAnkleAngleRate], true_obs_min_[rlsKneeAngleRate], true_obs_min_[rlsHipAngleRate], true_obs_min_[rlsTime];
  observation_max.resize(agent_obs_dim);
  observation_max << true_obs_max_[rlsAnkleAngle], true_obs_max_[rlsKneeAngle], true_obs_max_[rlsHipAngle],
      true_obs_max_[rlsAnkleAngleRate], true_obs_max_[rlsKneeAngleRate], true_obs_max_[rlsHipAngleRate], true_obs_max_[rlsTime];
  config.set("observation_min", observation_min);
  config.set("observation_max", observation_max);

  std::cout << observation_min << std::endl;
  std::cout << observation_max << std::endl;

  config.set("action_dims", action_dims_);
  config.set("action_min", VectorConstructor(-10.7, -10.7, -10.7));
  config.set("action_max", VectorConstructor( 10.7,  10.7,  10.7));
  config.set("reward_min", VectorConstructor(-1000));
  config.set("reward_max", VectorConstructor( 1000));
}

LeoSquatTaskFA *LeoSquatTaskFA::clone() const
{
  return new LeoSquatTaskFA(*this);
}

void LeoSquatTaskFA::start(int test, Vector *state) const
{
  *state = ConstantVector(rlsStateDim, 0);
  *state <<
         1.0586571916803691E+00,
        -2.1266836153365212E+00,
         1.0680264236561250E+00,
        -2.5999999999984957E-01,
        -0.0,
        -0.0,
        -0.0,
        -0.0,  // end of rlsDofDim
         0.0,  // rlsTime
         0.28, // rlsRefRootHeight, possible values 0.28 and 0.35
         ConstantVector(rlsStateDim - 2*rlsDofDim - 2, 0); // initialize the rest to zero

  (*state)[rlsRootZ] = 0.28;

  root_height_ = 0;
  squats_ = 0;
}

void LeoSquatTaskFA::observe(const Vector &state, Vector *obs, int *terminal) const
{
  grl_assert(state.size() == rlsStateDim);

  // arm is not actuated => exclude angle and angle rate from observations
//  std::cout << state << std::endl;

//  std::cout << state.block(0, rlsAnkleAngle, 1, rlsHipAngle-rlsAnkleAngle+1) << std::endl; // angle
//  std::cout << state.block(0, rlsAnkleAngleRate, 1, rlsHipAngleRate-rlsAnkleAngleRate+1) << std::endl; // angle rate
//  std::cout << state[rlsRefRootHeight] << std::endl; // direction indicator

  obs->resize(3+3+1);
  (*obs) << state.block(0, rlsAnkleAngle, 1, rlsHipAngle-rlsAnkleAngle+1),
            state.block(0, rlsAnkleAngleRate, 1, rlsHipAngleRate-rlsAnkleAngleRate+1),
            state[rlsRefRootZ];

  if (state[rlsTime] >= timeout_)
    *terminal = 1;
  else if (failed(state))
    *terminal = 2;
  else
    *terminal = 0;
}

void LeoSquatTaskFA::evaluate(const Vector &state, const Vector &action, const Vector &next, double *reward) const
{
  grl_assert(state.size() == rlsStateDim);
  grl_assert(action.size() == rlsDofDim || action.size() == rlsDofDim-1); // if auto-actuated, action is shorter
  grl_assert(next.size() == rlsStateDim);

  if (failed(next))
  {
    *reward = -1000000;
    return;
  }

  double cost = 0;

  // calculate support center from feet positions
  double suppport_center = 0.5 * (next[rlsLeftTipX] + next[rlsLeftHeelX]);

  // track: || root_z - h_ref ||_2^2
  cost +=  pow(100.0 * (next[rlsRootZ] - next[rlsRefRootZ]), 2);


  // track: || com_x,y - support center_x,y ||_2^2
  cost +=  pow( 50.00 * (next[rlsComX] - suppport_center), 2);

  cost +=  pow( 10.00 * next[rlsComVelocityX], 2);
  cost +=  pow( 10.00 * next[rlsComVelocityZ], 2);

  cost +=  pow( 100.00 * next[rlsAngularMomentumY], 2);

  // NOTE: sum of lower body angles is equal to angle between ground slope
  //       and torso. Minimizing deviation from zero keeps torso upright
  //       during motion execution.
  cost += pow(30.00 * ( next[rlsAnkleAngle] + next[rlsKneeAngle] + next[rlsHipAngle] - (0.15) ), 2); // desired torso angle

  // regularize torso
  // is this a good way for torso? Results in a very high penalty
  //cost += pow(60.00 * (next[rlsAnkleAngleRate] + next[rlsKneeAngleRate] + next[rlsHipAngleRate]), 2);

  // regularize: || qdot ||_2^2
  // res[res_cnt++] = 6.00 * sd[QDOTS["arm"]]; // arm
  cost += pow(6.00 * next[rlsHipAngleRate], 2); // hip_left
  cost += pow(6.00 * next[rlsKneeAngleRate], 2); // knee_left
  cost += pow(6.00 * next[rlsAnkleAngleRate], 2); // ankle_left

  // regularize: || u ||_2^2
  // res[res_cnt++] = 0.01 * u[TAUS["arm"]]; // arm
  cost += pow(0.01 * action[0], 2); // hip_left
  cost += pow(0.01 * action[1], 2); // knee_left
  cost += pow(0.01 * action[2], 2); // ankle_left

  double shaping = 0;

  double w = 60000.0;
  double F1, F0 = - fabs(w * (state[rlsRootZ] - state[rlsRefRootZ]));
  if (state[rlsRefRootZ] == next[rlsRefRootZ])
    F1 = - fabs(w * (next [rlsRootZ] - next [rlsRefRootZ]));
  else
    F1 = - fabs(w * (next [rlsRootZ] - state[rlsRefRootZ]));
  shaping += F1 - F0;


  // reward is a negative of cost
  *reward = -cost + shaping;

  // for progress report
  root_height_ = next[rlsRootZ];
  if (next[rlsRefRootZ] != state[rlsRefRootZ])
    squats_++;
}
