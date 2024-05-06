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
REGISTER_CONFIGURABLE(WMRTrajectoryTask)

void WMRDynamics::request(ConfigurationRequest *config)
{
  config->push_back(CRP("track", "Vehicle track (horizontal size)", t_));
  config->push_back(CRP("radius", "Wheel radius", r_));
  config->push_back(CRP("base", "Wheel base (front to back distance)", b_));
  config->push_back(CRP("length", "Caster wheel support length", l_));
  config->push_back(CRP("caster", "Include caster wheels in state", caster_, CRP::Configuration, 0, 1));
}

void WMRDynamics::configure(Configuration &config)
{
  t_ = config["track"];
  r_ = config["radius"];
  b_ = config["base"];
  l_ = config["length"];

  caster_ = config["caster"];
}

void WMRDynamics::reconfigure(const Configuration &config)
{
}

void WMRDynamics::eom(const Vector &state, const Vector &actuation, Vector *xd) const
{
  if (state.size() != 4 + 2*caster_ || actuation.size() != 2)
  {
    ERROR("State " << state << " or actuation " << actuation << " do not have correct size " << 4 + 2*caster_ << " / 2");
    throw Exception("dynamics/wmr requires a compatible task/wmr subclass");
  }
    
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

// WMRTrajectoryTask

void WMRTrajectoryTask::request(ConfigurationRequest *config)
{
  Task::request(config);

  config->push_back(CRP("v_linear", "Maximum linear velocity", v_linear_, CRP::Configuration, 0., 10.));
  config->push_back(CRP("v_angular", "Maximum angular velocity", v_angular_, CRP::Configuration, 0., 2*M_PI));
  config->push_back(CRP("sensor_pos", "Position of sensor bar w.r.t wheels", sensor_pos_, CRP::Configuration));
  config->push_back(CRP("sensor_width", "Width of sensor bar", sensor_width_, CRP::Configuration));
  config->push_back(CRP("sensor_elements", "Number of sensing elements on sensor bar", sensor_elements_, CRP::Configuration));
  config->push_back(CRP("start", "Starting position", start_, CRP::Configuration));

  config->push_back(CRP("trajectory", "mapping", "Image containing trajectory to follow", trajectory_));
}

void WMRTrajectoryTask::configure(Configuration &config)
{
  trajectory_ = (Mapping*)config["trajectory"].ptr();
  v_linear_ = config["v_linear"];
  v_angular_ = config["v_angular"];
  sensor_pos_ = config["sensor_pos"];
  sensor_width_ = config["sensor_width"];
  sensor_elements_ = config["sensor_elements"];
  start_ = config["start"].v();
  
  if (start_.size() != 3)
    throw bad_param("task/wmr/trajectory:start");
  
  config.set("observation_dims", 1);
  config.set("observation_min", VectorConstructor(-sensor_width_/2));
  config.set("observation_max", VectorConstructor( sensor_width_/2));
  config.set("action_dims", 2);
  config.set("action_min", VectorConstructor( 0.        , -v_angular_));
  config.set("action_max", VectorConstructor( v_linear_ ,  v_angular_));
  config.set("reward_min", -sensor_width_/2-10);
  config.set("reward_max", v_linear_);
}

void WMRTrajectoryTask::reconfigure(const Configuration &config)
{
}

void WMRTrajectoryTask::start(int test, Vector *state)
{
  state->resize(4);
  
  (*state)[0] = start_[0] + (test==0)*RandGen::getNormal(0, 0.01);
  (*state)[1] = start_[1] + (test==0)*RandGen::getNormal(0, 0.01);
  (*state)[2] = start_[2] + (test==0)*RandGen::getNormal(0, 0.1);
  (*state)[3] = 0;
}

void WMRTrajectoryTask::evaluate(const Vector &state, const Action &action, const Vector &next, double *reward) const
{
  Observation obs;
  int terminal;
  
  observe(next, &obs, &terminal);
  
  // Reward is linear velocity - distance to sensor 0 position
  *reward = action[0] - fabs(obs[0]);
  
  if (terminal)
    *reward -= 10;
}

void WMRTrajectoryTask::observe(const Vector &state, Observation *obs, int *terminal) const
{
  if (state.size() != 4)
  {
    ERROR("State " << state << " is not of expected size 4");
    throw Exception("task/wmr/trajectory requires compatible dynamics/wmr");
  }
  
  double theta = state[2];
  Eigen::Matrix2d rtheta(2, 2);
  rtheta << cos(theta), -sin(theta),
            sin(theta),  cos(theta);
  
  Eigen::Vector2d cur(2), pos(2);
  cur << state[0], state[1];
  
  pos[0] = sensor_pos_;
    
  //ERROR("robot: " << cur);
  double detect = 0, total = 0;
  for (size_t ii=0; ii < sensor_elements_; ++ii)
  {
    pos[1] = -sensor_width_/2 + ii*sensor_width_/(sensor_elements_-1);
    Vector world = cur + rtheta * pos;
    
    //ERROR(world);
    
    // Read mapping at world coordinate.
    Vector res; 
    double d = trajectory_->read(world, &res);
    detect += pos[1] * d;
    total += d;
  }
  
  if (total != 0)
    detect /= total;
  
  obs->v.resize(1);
  obs->v[0] = detect;
  
  // Terminal if sensor leaves track completely
  if (total == 0)
  {
    obs->absorbing = true;
    *terminal = 1;
  }
  else if (state[3] > 10)
  {
    obs->absorbing = false;
    *terminal = 1;
  }
  else
  {
    obs->absorbing = false;
    *terminal = false;
  }
}

bool WMRTrajectoryTask::invert(const Observation &obs, Vector *state, double time) const
{
  return false;
}
