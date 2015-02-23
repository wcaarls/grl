/** \file cart_pole.cpp
 * \brief Cart-Pole environment source file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-02-02
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

#include <grl/environments/cart_pole.h>

using namespace grl;

REGISTER_CONFIGURABLE(CartPoleDynamics)
REGISTER_CONFIGURABLE(CartPoleSwingupTask)
REGISTER_CONFIGURABLE(CartPoleBalancingTask)

void CartPoleDynamics::request(ConfigurationRequest *config)
{
}

void CartPoleDynamics::configure(Configuration &config)
{
  g_ = 9.8,
  mass_cart_ = 1.0;
  mass_pole_ = 0.1;
  total_mass_ = mass_cart_ + mass_pole_;
  length_ = 0.5;
  pole_mass_length_ = mass_pole_ * length_;
}

void CartPoleDynamics::reconfigure(const Configuration &config)
{
}

CartPoleDynamics *CartPoleDynamics::clone() const
{
  return new CartPoleDynamics(*this);
}

void CartPoleDynamics::eom(const Vector &state, const Vector &action, Vector *xd) const
{
  if (state.size() != 5 || action.size() != 1)
    throw Exception("dynamics/cart_pole requires a task/cart_pole subclass");

  double acc,thetaacc,costheta,sintheta,temp;

  costheta = cos(static_cast<double>(state[2]));
  sintheta = sin(static_cast<double>(state[2]));
 
  temp = (action[0] + pole_mass_length_ * state[3] * state[3] * sintheta) / total_mass_;
 
  thetaacc = (g_ * sintheta - costheta * temp) /
             (length_ * ((4./3.) - mass_pole_ * costheta * costheta / total_mass_));
 
  acc  = temp - pole_mass_length_ * thetaacc * costheta / total_mass_;

  xd->resize(5);
  (*xd)[0] = state[1];
  (*xd)[1] = acc;
  (*xd)[2] = state[3];
  (*xd)[3] = thetaacc;
  (*xd)[4] = 1;
}

void CartPoleSwingupTask::request(ConfigurationRequest *config)
{
  Task::request(config);

  config->push_back(CRP("timeout", "Episode timeout", T_, CRP::Configuration, 0., DBL_MAX));
  config->push_back(CRP("randomization", "Start state randomization", randomization_, CRP::Online, 0, 1));
  config->push_back(CRP("shaping", "Whether to use reward shaping", shaping_, CRP::Configuration, 0, 1));
  config->push_back(CRP("gamma", "Discount rate for reward shaping", gamma_, CRP::Configuration, 0., 1.));
}

void CartPoleSwingupTask::configure(Configuration &config)
{
  randomization_ = config["randomization"];
  shaping_ = config["shaping"];
  gamma_ = config["gamma"];
  T_ = config["timeout"];

  config.set("observation_dims", 4);
  config.set("observation_min", VectorConstructor(-2.4, -10.0, 0.    , -10*M_PI));
  config.set("observation_max", VectorConstructor( 2.4,  10.0, 2*M_PI,  10*M_PI));
  config.set("action_dims", 1);
  config.set("action_min", VectorConstructor(-15.));
  config.set("action_max", VectorConstructor( 15.));
  
  if (shaping_)
  {
    config.set("reward_min", -2*pow(2.4, 2) - 0.1*pow(10, 2) - pow(M_PI, 2) - 0.1*pow(10*M_PI, 2) + 1 - 100);
    config.set("reward_max", 0);
  }
  else
  {
    config.set("reward_min", -2*pow(2.4, 2) - 0.1*pow(10, 2) - pow(M_PI, 2) - 0.1*pow(10*M_PI, 2) - 10000);
    config.set("reward_max", 0);
  }
}

void CartPoleSwingupTask::reconfigure(const Configuration &config)
{
  config.get("randomization", randomization_);
}

CartPoleSwingupTask *CartPoleSwingupTask::clone() const
{
  return new CartPoleSwingupTask(*this);
}

void CartPoleSwingupTask::start(Vector *state) const
{
  state->resize(5);

  (*state)[0] = 0;
  (*state)[1] = 0;
  (*state)[2] = M_PI+randomization_*((RandGen::get()*0.1)-0.05);
  (*state)[3] = 0;
  (*state)[4] = 0;
}

void CartPoleSwingupTask::observe(const Vector &state, Vector *obs, int *terminal) const
{
  if (state.size() != 5)
    throw Exception("task/cart_pole/swingup requires dynamics/cart_pole");

  double a = fmod(state[2]+M_PI, 2*M_PI);
  if (a < 0) a += 2*M_PI;
  
  obs->resize(4);
  (*obs)[0] = state[0];
  (*obs)[1] = state[1];
  (*obs)[2] = a;
  (*obs)[3] = state[3];

  if (failed(state))
    *terminal = 2;
  else if (state[4] > T_)
    *terminal = 1;
  else
    *terminal = 0;
}

void CartPoleSwingupTask::evaluate(const Vector &state, const Vector &action, const Vector &next, double *reward) const
{
  if (state.size() != 5 || action.size() != 1 || next.size() != 5)
    throw Exception("task/cart_pole/swingup requires dynamics/cart_pole");

  if (shaping_)
    *reward = gamma_*potential(next) - potential(state) + succeeded(next) - failed(next)*100;
  else
    *reward = potential(next) - failed(next)*10000;
}

bool CartPoleSwingupTask::invert(const Vector &obs, Vector *state) const
{
  *state = obs;
  (*state)[3] -= M_PI;
  state->push_back(0.);
  
  return true;
}

bool CartPoleSwingupTask::failed(const Vector &state) const
{
  return fabs(state[0]) > 2.4;
}

bool CartPoleSwingupTask::succeeded(const Vector &state) const
{
  double a = fmod(fabs(state[2]), 2*M_PI);
  if (a > M_PI) a -= 2*M_PI;   

  return fabs(state[0]) < 0.1 && fabs(state[1]) < 0.5 &&
         fabs(a) < 5*M_PI/180 && fabs(state[3]) < 25*M_PI/180;
}

double CartPoleSwingupTask::potential(const Vector &state) const
{
  double a = fmod(fabs(state[2]), 2*M_PI);
  if (a > M_PI) a -= 2*M_PI;

  return -2*pow(state[0], 2) - 0.1*pow(state[1], 2) - pow(a, 2) - 0.1*pow(state[3], 2);
}

void CartPoleBalancingTask::request(ConfigurationRequest *config)
{
  Task::request(config);

  config->push_back(CRP("timeout", "Episode timeout", T_, CRP::Configuration, 0.));
}

void CartPoleBalancingTask::configure(Configuration &config)
{
  T_ = config["timeout"];

  config.set("observation_dims", 4);
  config.set("observation_min", VectorConstructor(-2.4, -5.0, M_PI-12*M_PI/180, -M_PI));
  config.set("observation_max", VectorConstructor( 2.4,  5.0, M_PI+12*M_PI/180,  M_PI));
  config.set("action_dims", 1);
  config.set("action_min", VectorConstructor(-15.));
  config.set("action_max", VectorConstructor( 15.));
  config.set("reward_min", 0);
  config.set("reward_max", 1);
}

void CartPoleBalancingTask::reconfigure(const Configuration &config)
{
}

CartPoleBalancingTask *CartPoleBalancingTask::clone() const
{
  return new CartPoleBalancingTask(*this);
}

void CartPoleBalancingTask::start(Vector *state) const
{
  state->resize(5);

  (*state)[0] = 0;
  (*state)[1] = 0;
  (*state)[2] = (RandGen::get()*0.1)-0.05;
  (*state)[3] = 0;
  (*state)[4] = 0;
}

void CartPoleBalancingTask::observe(const Vector &state, Vector *obs, int *terminal) const
{
  if (state.size() != 5)
    throw Exception("task/cart_pole/balacing requires dynamics/cart_pole");

  obs->resize(4);
  (*obs)[0] = state[0];
  (*obs)[1] = state[1];
  (*obs)[2] = state[2];
  (*obs)[3] = state[3];

  if (failed(state))
    *terminal = 2;
  else if (state[4] > T_)
    *terminal = 1;
  else
    *terminal = 0;
}

void CartPoleBalancingTask::evaluate(const Vector &state, const Vector &action, const Vector &next, double *reward) const
{
  if (state.size() != 5 || action.size() != 1 || next.size() != 5)
    throw Exception("task/cart_pole/balancing requires dynamics/cart_pole");

  *reward = 1 - failed(next);
}

bool CartPoleBalancingTask::invert(const Vector &obs, Vector *state) const
{
  *state = obs;
  state->push_back(0.);
  
  return true;
}

bool CartPoleBalancingTask::failed(const Vector &state) const
{
  return fabs(state[0]) > 2.4 || fabs(state[2]) > 12*M_PI/180;
}
