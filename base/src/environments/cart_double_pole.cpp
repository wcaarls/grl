/** \file cart_double_pole.cpp
 * \brief Cart-Double-Pole environment source file.
 *
 * Based on the equations of motion from Zhong and Rock,
 * "Energy and passivity based control of the double inverted pendulum on a cart",
 * Proceedings of the 2001 IEEE International Conference on Control Applications,
 * 2001  
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2016-11-08
 *
 * \copyright \verbatim
 * Copyright (c) 2016, Wouter Caarls
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

#include <grl/environments/cart_double_pole.h>

using namespace grl;

REGISTER_CONFIGURABLE(CartDoublePoleDynamics)
REGISTER_CONFIGURABLE(CartDoublePoleSwingupTask)
REGISTER_CONFIGURABLE(CartDoublePoleBalancingTask)
REGISTER_CONFIGURABLE(CartDoublePoleRegulatorTask)

void CartDoublePoleDynamics::request(ConfigurationRequest *config)
{
}

void CartDoublePoleDynamics::configure(Configuration &config)
{
  m_  = 0.5,              // Mass of cart
  m1_ = 0.5,              // Mass of first pole
  m2_ = 0.5;              // Mass of second pole
  l1_ = 0.3,              // CoM of first pole
  l2_ = 0.3;              // CoM of second pole
  b_  = 0.1;              // Friction between cart and ground
  g_  = 9.82;             // Gravitational constant

  L1_ = 2*l1_,             // Length of first pole
  L2_ = 2*l2_;             // Length of second pole
  
  double J1 = m1_*L1_*L1_/3, // Inertia of first pole
         J2 = m2_*L2_*L2_/3; // Inertia of second pole

  // Equation (5)
  h1_ = m_ + m1_ + m2_;
  h2_ = m1_*l1_ + m2_*L1_;
  h3_ = m2_*l2_;
  h4_ = m1_*l1_*l1_ + m2_*L1_*L1_ + J1;
  h5_ = m2_*l2_*L1_;
  h6_ = m2_*l2_*l2_ + J2;
  h7_ = m1_*l1_*g_ + m2_*L1_*g_;
  h8_ = m2_*l2_*g_;
}

void CartDoublePoleDynamics::reconfigure(const Configuration &config)
{
}

CartDoublePoleDynamics *CartDoublePoleDynamics::clone() const
{
  return new CartDoublePoleDynamics(*this);
}

/* Pole angles are relative to vertical */
void CartDoublePoleDynamics::eom(const Vector &state, const Vector &action, Vector *xd) const
{
  if (state.size() != 7 || action.size() != 1)
    throw Exception("dynamics/cart_double_pole requires a task/cart_double_pole subclass");

  Eigen::Matrix3d M, C;
  Eigen::Vector3d qd(state[3], state[4], state[5]),
                  g, u;
 
  double c1 = cos(state[1]), c2 = cos(state[2]),
         s1 = sin(state[1]), s2 = sin(state[2]),
         c12 = cos(state[1] - state[2]),
         s12 = sin(state[1] - state[2]),
         f = action[0];
        
  M << h1_,     h2_*c1,         h3_*c2,
       h2_*c1,  h4_,            h5_*c12,
       h3_*c2,  h5_*c12,        h6_;
  C << 0,      -h2_*qd[1]*s1,  -h3_*qd[2]*s2,
       0,       0,              h5_*qd[2]*s12,
       0,      -h5_*qd[1]*s12,  0;
  g << 0,      -h7_*s1,        -h8_*s2;
  u << f,       0,              0;
 
  Eigen::Vector3d qdd = M.inverse()*(u - (C*qd) - g);
 
  xd->resize(7);
  (*xd)[0] = qd[0];
  (*xd)[1] = qd[1];
  (*xd)[2] = qd[2];
  (*xd)[3] = qdd[0];
  (*xd)[4] = qdd[1];
  (*xd)[5] = qdd[2];
  (*xd)[6] = 1;
  
  // Simulate end stop
  // NOTE: Cannot reset position or velocity here, so just keep them from getting larger
  if (state[0] > 2.4 && state[3] > 0)
  {
    (*xd)[0] = 0;
    if ((*xd)[3] > 0)
      (*xd)[3] = 0;
  }
  else if (state[0] < -2.4 && state[3] < 0)
  {
    (*xd)[0] = 0;
    if ((*xd)[3] < 0)
      (*xd)[3] = 0;
  }
}

void CartDoublePoleSwingupTask::request(ConfigurationRequest *config)
{
  Task::request(config);

  config->push_back(CRP("timeout", "Episode timeout", T_, CRP::Configuration, 0., DBL_MAX));
}

void CartDoublePoleSwingupTask::configure(Configuration &config)
{
  T_ = config["timeout"];

  config.set("observation_dims", 6);
  config.set("observation_min", VectorConstructor(-2.4, 0.,     0.    , -10.0, -5*M_PI, -5*M_PI));
  config.set("observation_max", VectorConstructor( 2.4, 2*M_PI, 2*M_PI,  10.0,  5*M_PI,  5*M_PI));
  config.set("action_dims", 1);
  config.set("action_min", VectorConstructor(-20.));
  config.set("action_max", VectorConstructor( 20.));
  
  config.set("reward_min", -2*pow(2.4, 2) - 0.1*pow(10, 2) - 2*pow(M_PI, 2) - 0.2*pow(5*M_PI, 2));
  config.set("reward_max", 0);
}

void CartDoublePoleSwingupTask::reconfigure(const Configuration &config)
{
}

CartDoublePoleSwingupTask *CartDoublePoleSwingupTask::clone() const
{
  return new CartDoublePoleSwingupTask(*this);
}

void CartDoublePoleSwingupTask::start(int test, Vector *state) const
{
  state->resize(7);

  (*state)[0] = 0;
  (*state)[1] = M_PI+((RandGen::get()*0.1)-0.05);
  (*state)[2] = 0;
  (*state)[3] = 0;
  (*state)[4] = 0;
  (*state)[5] = 0;
  (*state)[6] = 0;
}

void CartDoublePoleSwingupTask::observe(const Vector &state, Vector *obs, int *terminal) const
{
  if (state.size() != 5)
    throw Exception("task/cart_double_pole/swingup requires dynamics/cart_double_pole");

  double a = fmod(state[1]+M_PI, 2*M_PI);
  if (a < 0) a += 2*M_PI;
  
  double a2 = fmod(state[2]+M_PI, 2*M_PI);
  if (a2 < 0) a2 += 2*M_PI;
  
  obs->resize(6);
  (*obs)[0] = state[0];
  (*obs)[1] = a;
  (*obs)[2] = a2;
  (*obs)[3] = state[3];
  (*obs)[3] = state[4];
  (*obs)[3] = state[5];

  if (state[6] > T_)
    *terminal = 1;
  else
    *terminal = 0;
}

void CartDoublePoleSwingupTask::evaluate(const Vector &state, const Vector &action, const Vector &next, double *reward) const
{
  if (state.size() != 7 || action.size() != 1 || next.size() != 7)
    throw Exception("task/cart_double_pole/swingup requires dynamics/cart_double_pole");

  double a = fmod(state[1]+M_PI, 2*M_PI);
  if (a < 0) a += 2*M_PI;
  
  double a2 = fmod(state[2]+M_PI, 2*M_PI);
  if (a2 < 0) a2 += 2*M_PI;
  
  *reward = -2*pow(state[0], 2) - 0.1*pow(state[3], 2) -pow(a, 2) - 0.1*pow(state[4], 2) - pow(a2, 2) - 0.1*pow(state[5], 2);
}

bool CartDoublePoleSwingupTask::invert(const Vector &obs, Vector *state) const
{
  *state = obs;
  (*state)[1] -= M_PI;
  *state = extend(*state, VectorConstructor(0.));
  
  return true;
}

Matrix CartDoublePoleSwingupTask::rewardHessian(const Vector &state, const Vector &action) const
{
  Vector d;
  d << -2., -1., -1., -0.1, -0.1, -0.1, -0.01;

  return diagonal(d);
}

void CartDoublePoleBalancingTask::request(ConfigurationRequest *config)
{
  Task::request(config);

  config->push_back(CRP("timeout", "Episode timeout", T_, CRP::Configuration, 0., DBL_MAX));
}

void CartDoublePoleBalancingTask::configure(Configuration &config)
{
  T_ = config["timeout"];

  config.set("observation_dims", 6);
  config.set("observation_min", VectorConstructor(-2.4, -0.7, -0.7, -5.0, -5.0, -5.0));
  config.set("observation_max", VectorConstructor( 2.4,  0.7,  0.7,  5.0,  5.0,  5.0));
  config.set("action_dims", 1);
  config.set("action_min", VectorConstructor(-20.));
  config.set("action_max", VectorConstructor( 20.));
  config.set("reward_min", 0);
  config.set("reward_max", 1);
}

void CartDoublePoleBalancingTask::reconfigure(const Configuration &config)
{
}

CartDoublePoleBalancingTask *CartDoublePoleBalancingTask::clone() const
{
  return new CartDoublePoleBalancingTask(*this);
}

void CartDoublePoleBalancingTask::start(int test, Vector *state) const
{
  state->resize(7);

  (*state)[0] = 0;
  (*state)[1] = (RandGen::get()*0.01)-0.005;
  (*state)[2] = 0;
  (*state)[3] = 0;
  (*state)[4] = 0;
  (*state)[5] = 0;
  (*state)[6] = 0;
}

void CartDoublePoleBalancingTask::observe(const Vector &state, Vector *obs, int *terminal) const
{
  if (state.size() != 7)
    throw Exception("task/cart_double_pole/balancing requires dynamics/cart_double_pole");

  obs->resize(6);
  (*obs)[0] = state[0];
  (*obs)[1] = state[1];
  (*obs)[2] = state[2];
  (*obs)[3] = state[3];
  (*obs)[4] = state[4];
  (*obs)[5] = state[5];

  if (failed(state))
    *terminal = 2;
  else if (state[6] > T_)
    *terminal = 1;
  else
    *terminal = 0;
}

void CartDoublePoleBalancingTask::evaluate(const Vector &state, const Vector &action, const Vector &next, double *reward) const
{
  if (state.size() != 7 || action.size() != 1 || next.size() != 7)
    throw Exception("task/cart_double_pole/balancing requires dynamics/cart_double_pole");

  *reward = 1 - failed(next);
}

bool CartDoublePoleBalancingTask::invert(const Vector &obs, Vector *state) const
{
  *state = obs;
  *state = extend(*state, VectorConstructor(0.));
  
  return true;
}

bool CartDoublePoleBalancingTask::failed(const Vector &state) const
{
  return fabs(state[0]) > 2.4 || fabs(state[1]) > 0.7 || fabs(state[2]) > 0.7;
}

// Regulator

void CartDoublePoleRegulatorTask::request(ConfigurationRequest *config)
{
  RegulatorTask::request(config);

  config->push_back(CRP("timeout", "Episode timeout", T_, CRP::Configuration, 0., DBL_MAX));
}

void CartDoublePoleRegulatorTask::configure(Configuration &config)
{
  RegulatorTask::configure(config);
  
  T_ = config["timeout"];

  config.set("observation_min", VectorConstructor(-2.4, -M_PI, -M_PI, -10.0, -5*M_PI, -5*M_PI));
  config.set("observation_max", VectorConstructor( 2.4,  M_PI,  M_PI,  10.0,  5*M_PI,  5*M_PI));
  config.set("action_min", VectorConstructor(-20.));
  config.set("action_max", VectorConstructor( 20.));
}

void CartDoublePoleRegulatorTask::reconfigure(const Configuration &config)
{
  RegulatorTask::reconfigure(config);
}

CartDoublePoleRegulatorTask *CartDoublePoleRegulatorTask::clone() const
{
  return new CartDoublePoleRegulatorTask(*this);
}

void CartDoublePoleRegulatorTask::observe(const Vector &state, Vector *obs, int *terminal) const
{
  if (state.size() != 7)
    throw Exception("task/cart_double_pole/regulator requires dynamics/cart_double_pole");

  obs->resize(6);
  (*obs)[0] = state[0];
  (*obs)[1] = state[1];
  (*obs)[2] = state[2];
  (*obs)[3] = state[3];
  (*obs)[4] = state[4];
  (*obs)[5] = state[5];

  if (state[6] > T_)
    *terminal = 1;
  else
    *terminal = 0;
}

bool CartDoublePoleRegulatorTask::invert(const Vector &obs, Vector *state) const
{
  *state = obs;
  *state = extend(*state, VectorConstructor(0.));
  
  return true;
}
