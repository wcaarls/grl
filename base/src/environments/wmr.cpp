/** \file wmr.cpp
 * \brief Wheeled mobile robot environment source file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2023-03-10
 *
 * \copyright \verbatim
 * Copyright (c) 2023, Wouter Caarls
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

#include <grl/environments/wmr.h>

using namespace grl;

REGISTER_CONFIGURABLE(WMRDynamics)
REGISTER_CONFIGURABLE(WMRRegulatorTask)
REGISTER_CONFIGURABLE(WMRCasterRegulatorTask)

void WMRDynamics::request(ConfigurationRequest *config)
{
  config->push_back(CRP("caster", "Include caster wheels", caster_, CRP::Configuration, 0, 1));
}

void WMRDynamics::configure(Configuration &config)
{
  t_ = 1.0; // Track (horizontal size)
  r_ = 0.1; // Wheel radius
  l_ = 0.2; // Caster wheel support length
  b_ = 1.0; // Wheelbase (front to back distance)

  caster_ = config["caster"];
}

void WMRDynamics::reconfigure(const Configuration &config)
{
}

void WMRDynamics::eom(const Vector &state, const Vector &actuation, Vector *xd) const
{
  if (state.size() != 4 + 2*caster_ || actuation.size() != 2)
    throw Exception("dynamics/wmr requires a compatible task/wmr subclass");
    
  xd->resize(4 + 2*caster_);
  (*xd)[0] = actuation[0]*cos(state[2]);
  (*xd)[1] = actuation[0]*sin(state[2]);
  (*xd)[2] = actuation[1];
  
  if (caster_)
  {
    double t2 = t_/2;
    double dtheta = actuation[1];

    // Derivative of rotation matrix around 0
    // NOTE: caster wheel angles are relative to robot base
    // d cos theta   d cos theta   d theta
    // ----------- = ----------- * -------
    // d t           d theta        d t
    Eigen::Matrix2d drtheta(2, 2);
    drtheta << 0,      -dtheta,
               dtheta, 0;
               
    // Base-relative velocities at the caster wheel mounts
    Eigen::Vector2d dp { actuation[0], 0 };  
    Eigen::Vector2d dp1 = drtheta * Eigen::Vector2d { -b_,  t2 } + dp; // left
    Eigen::Vector2d dp2 = drtheta * Eigen::Vector2d { -b_, -t2 } + dp; // right

    // https://eul.ink/robotics/Kinematics%20of%20Caster%20Wheels/index.html
    // However, we define zero angle as the wheel trailing behind the robot
    (*xd)[3] = (dp1[1]*cos(state[3]) - dp1[0]*sin(state[3]))/l_;
    (*xd)[4] = (dp2[1]*cos(state[4]) - dp2[0]*sin(state[4]))/l_;
  }
  
  (*xd)[3+2*caster_] = 1;
}

// Regulator without casters

void WMRRegulatorTask::request(ConfigurationRequest *config)
{
  RegulatorTask::request(config);
  
  config->push_back(CRP("v_linear", "Maximum linear velocity", v_linear_, CRP::Configuration, 0., 10.));
  config->push_back(CRP("v_angular", "Maximum angular velocity", v_angular_, CRP::Configuration, 0., 2*M_PI));
}

void WMRRegulatorTask::configure(Configuration &config)
{
  RegulatorTask::configure(config);
  
  v_linear_ = config["v_linear"];
  v_angular_ = config["v_angular"];
  
  if (q_.size() != 3)
    throw bad_param("task/wmr/regulator:q");
  if (r_.size() != 2)
    throw bad_param("task/wmr/regulator:r");

  config.set("observation_min", VectorConstructor(-10, -10, -M_PI));
  config.set("observation_max", VectorConstructor( 10,  10,  M_PI));
  config.set("action_min", VectorConstructor(-v_linear_ , -v_angular_));
  config.set("action_max", VectorConstructor( v_linear_ ,  v_angular_));
}

void WMRRegulatorTask::reconfigure(const Configuration &config)
{
  RegulatorTask::reconfigure(config);
}


void WMRRegulatorTask::evaluate(const Vector &state, const Action &action, const Vector &next, double *reward) const
{
  // Bound angular error to pi 
  Vector _state = state, _next = next;
  
  _state[2] = fmod(_state[2], M_PI);
  _next[2] = fmod(_next[2], M_PI);
  
  RegulatorTask::evaluate(_state, action, _next, reward);
}

void WMRRegulatorTask::observe(const Vector &state, Observation *obs, int *terminal) const
{
  RegulatorTask::observe(state, obs, terminal);

  if (state.size() != 4)
    throw Exception("task/wmr/regulator requires compatible dynamics/wmr");
    
  obs->v.resize(3);
  for (size_t ii=0; ii != 2; ++ii)
    (*obs)[ii] = state[ii];
    
  for (size_t ii=2; ii != 3; ++ii)
  {
    double a = fmod(state[ii]+M_PI, 2*M_PI);
    if (a < 0) a += 2*M_PI;
    a -= M_PI;
  
    (*obs)[ii] = a;
  }
    
  obs->absorbing = false;
  
  if (fabs(state[0]) >= 10 || fabs(state[1]) >= 10)
    *terminal = 1;
}

bool WMRRegulatorTask::invert(const Observation &obs, Vector *state, double time) const
{
  *state = extend(obs, VectorConstructor(time));
  
  return true;
}

// Regulator with casters

void WMRCasterRegulatorTask::request(ConfigurationRequest *config)
{
  RegulatorTask::request(config);
  
  config->push_back(CRP("v_linear", "Maximum linear velocity", v_linear_, CRP::Configuration, 0., 10.));
  config->push_back(CRP("v_angular", "Maximum angular velocity", v_angular_, CRP::Configuration, 0., 2*M_PI));
}

void WMRCasterRegulatorTask::configure(Configuration &config)
{
  RegulatorTask::configure(config);
  
  v_linear_ = config["v_linear"];
  v_angular_ = config["v_angular"];
  
  if (q_.size() != 5)
    throw bad_param("task/wmr/regulator:q");
  if (r_.size() != 2)
    throw bad_param("task/wmr/regulator:r");

  config.set("observation_min", VectorConstructor(-10, -10, -M_PI, -M_PI, -M_PI));
  config.set("observation_max", VectorConstructor( 10,  10,  M_PI,  M_PI,  M_PI));
  config.set("action_min", VectorConstructor(-v_linear_ , -v_angular_));
  config.set("action_max", VectorConstructor( v_linear_ ,  v_angular_));
}

void WMRCasterRegulatorTask::reconfigure(const Configuration &config)
{
  RegulatorTask::reconfigure(config);
}


void WMRCasterRegulatorTask::evaluate(const Vector &state, const Action &action, const Vector &next, double *reward) const
{
  // Bound angular error to pi 
  Vector _state = state, _next = next;
  
  for (size_t ii=2; ii != 5; ++ii)
  {
    _state[ii] = fmod(_state[ii], M_PI);
    _next[ii] = fmod(_next[ii], M_PI);
  }
  
  RegulatorTask::evaluate(_state, action, _next, reward);
}

void WMRCasterRegulatorTask::observe(const Vector &state, Observation *obs, int *terminal) const
{
  RegulatorTask::observe(state, obs, terminal);

  if (state.size() != 6)
    throw Exception("task/wmr/regulator requires compatible dynamics/wmr");
    
  obs->v.resize(5);
  for (size_t ii=0; ii != 2; ++ii)
    (*obs)[ii] = state[ii];
    
  for (size_t ii=2; ii != 5; ++ii)
  {
    double a = fmod(state[ii]+M_PI, 2*M_PI);
    if (a < 0) a += 2*M_PI;
    a -= M_PI;
  
    (*obs)[ii] = a;
  }
    
  obs->absorbing = false;
  
  if (fabs(state[0]) >= 10 || fabs(state[1]) >= 10)
    *terminal = 1;
}

bool WMRCasterRegulatorTask::invert(const Observation &obs, Vector *state, double time) const
{
  *state = extend(obs, VectorConstructor(time));
  
  return true;
}
