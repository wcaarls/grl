/** \file pinball.cpp
 * \brief Pinball environment source file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-08-12
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

#include <grl/environments/pinball.h>

using namespace grl;

REGISTER_CONFIGURABLE(PinballModel)
REGISTER_CONFIGURABLE(PinballMovementTask)
REGISTER_CONFIGURABLE(PinballRegulatorTask)

void PinballModel::request(ConfigurationRequest *config)
{
  config->push_back(CRP("control_step", "double.control_step", "Control step time", tau_, CRP::Configuration, 0.001, DBL_MAX));
  config->push_back(CRP("integration_steps", "Number of integration steps per control step", (int)steps_, CRP::Configuration, 1));
  config->push_back(CRP("restitution", "Coefficient of restitution", restitution_, CRP::Configuration, 0., 1.));
  config->push_back(CRP("radius", "Ball radius", radius_, CRP::Configuration, 0.001, DBL_MAX));
  config->push_back(CRP("maze", "Maze ID", maze_, CRP::Configuration, 0, 1));
}

void PinballModel::configure(Configuration &config)
{
  tau_ = config["control_step"];
  steps_ = config["integration_steps"];
  restitution_ = config["restitution"];
  radius_ = config["radius"];
  maze_ = config["maze"];
  
  if (maze_ == 0)
  {
    Obstacle w;
    w.addPoint(Point(0, 0));
    w.addPoint(Point(0, 1));
    w.addPoint(Point(1, 1));
    w.addPoint(Point(1, 0));
    obstacles_.push_back(w);
    
    Obstacle o1;
    o1.addPoint(Point(0.2, 0.0));
    o1.addPoint(Point(0.4, 0.0));
    o1.addPoint(Point(0.4, 0.8));
    o1.addPoint(Point(0.2, 0.8));
    obstacles_.push_back(o1);
    
    Obstacle o2;
    o2.addPoint(Point(0.6, 0.2));
    o2.addPoint(Point(0.8, 0.2));
    o2.addPoint(Point(0.8, 1.0));
    o2.addPoint(Point(0.6, 1.0));
    obstacles_.push_back(o2);
  }
  else if (maze_ == 1)
  {
    Obstacle w;
    w.addPoint(Point(0, 0));
    w.addPoint(Point(0, 1));
    w.addPoint(Point(1, 1));
    w.addPoint(Point(1, 0));
    obstacles_.push_back(w);
    
    Obstacle o1;
    o1.addPoint(Point(0.2, 0.0));
    o1.addPoint(Point(0.3, 0.0));
    o1.addPoint(Point(0.3, 0.7));
    o1.addPoint(Point(0.5, 0.7));
    o1.addPoint(Point(0.5, 0.8));
    o1.addPoint(Point(0.2, 0.8));
    obstacles_.push_back(o1);

    Obstacle o2;
    o2.addPoint(Point(0.5, 0.2));
    o2.addPoint(Point(0.8, 0.2));
    o2.addPoint(Point(0.8, 1.0));
    o2.addPoint(Point(0.7, 1.0));
    o2.addPoint(Point(0.7, 0.3));
    o2.addPoint(Point(0.5, 0.3));
    obstacles_.push_back(o2);
  }
}

void PinballModel::reconfigure(const Configuration &config)
{
}

double PinballModel::step(const Vector &state, const Vector &actuation, Vector *next) const
{
  Ball b = Ball(Point(state[siX], state[siY]), Point(state[siXd], state[siYd]), radius_);
  double h = tau_/steps_;
  bool collision = false;

  for (int ii=0; ii < steps_; ++ii)
  {
    b.roll(Point(actuation[aiXdd], actuation[aiYdd]), h);
    for (size_t jj=0; jj < obstacles_.size(); ++jj)
      collision |= obstacles_[jj].collide(b, restitution_);
  }

  *next = VectorConstructor(b.pos().x(), b.pos().y(), b.vel().x(), b.vel().y(), state[siTime]+tau_);
  
  return tau_;
}

void PinballMovementTask::request(ConfigurationRequest *config)
{
  Task::request(config);

  config->push_back(CRP("tolerance", "Goal tolerance", tolerance_, CRP::Configuration, 0.001, DBL_MAX));
}

void PinballMovementTask::configure(Configuration &config)
{
  tolerance_ = config["tolerance"];

  config.set("observation_dims", 4);
  config.set("observation_min", VectorConstructor(0, 0, -2, -2));
  config.set("observation_max", VectorConstructor(1, 1, 2, 2));
  config.set("action_dims", 2);
  config.set("action_min", VectorConstructor(-1, -1));
  config.set("action_max", VectorConstructor(1, 1));
  config.set("reward_min", -1);
  config.set("reward_max", 100);
}

void PinballMovementTask::reconfigure(const Configuration &config)
{
}

void PinballMovementTask::start(int test, Vector *state) const
{
  *state = VectorConstructor(0.1, 0.1, 0., 0., 0.);
}

void PinballMovementTask::observe(const Vector &state, Observation *obs, int *terminal) const
{
  if (state.size() != 5)
    throw Exception("task/pinball/movement requires model/pinball");
    
  obs->v.resize(4);
  for (size_t ii=0; ii < 4; ++ii)
    (*obs)[ii] = state[ii];
  obs->absorbing = false;
      
  if (succeeded(state))
  {
    *terminal = 2;
    obs->absorbing = true;
  }
  else if (state[PinballModel::siTime] > 10)
    *terminal = 1;
  else
    *terminal = 0;
}

void PinballMovementTask::evaluate(const Vector &state, const Action &action, const Vector &next, double *reward) const
{
  if (state.size() != 5 || action.size() != 2 || next.size() != 5)
    throw Exception("task/pinball/movement requires model/pinball");

  if (succeeded(next))
    *reward = 100;
  else
    *reward = -1;
}

bool PinballMovementTask::invert(const Observation &obs, Vector *state, double time) const
{
  state->resize(5);
  for (size_t ii=0; ii < 4; ++ii)
    (*state)[ii] = obs[ii];
  (*state)[PinballModel::siTime] = time;
  
  return true;
}

bool PinballMovementTask::succeeded(const Vector &state) const
{
  if ((fabs(state[PinballModel::siX] - 0.9) < tolerance_ && (fabs(state[PinballModel::siY] - 0.9) < tolerance_)))
    return true;
  else
    return false;
}

// Regulator

void PinballRegulatorTask::request(ConfigurationRequest *config)
{
  RegulatorTask::request(config);
}

void PinballRegulatorTask::configure(Configuration &config)
{
  RegulatorTask::configure(config);
  
  if (q_.size() != 4)
    throw bad_param("task/pinball/regulator:q");
  if (r_.size() != 2)
    throw bad_param("task/pinball/regulator:r");

  config.set("observation_min", VectorConstructor(0, 0, -2, -2));
  config.set("observation_max", VectorConstructor(1, 1,  2,  2));
  config.set("action_min", VectorConstructor(-1, -1));
  config.set("action_max", VectorConstructor(1, 1));
}

void PinballRegulatorTask::reconfigure(const Configuration &config)
{
  RegulatorTask::reconfigure(config);
}

void PinballRegulatorTask::observe(const Vector &state, Observation *obs, int *terminal) const
{
  if (state.size() != 5)
    throw Exception("task/pinball/regulator requires dynamics/pinball");
    
  obs->v.resize(4);
  for (size_t ii=0; ii < 4; ++ii)
    (*obs)[ii] = state[ii];
  obs->absorbing = false;

  *terminal = state[4] > 20;
}

bool PinballRegulatorTask::invert(const Observation &obs, Vector *state, double time) const
{
  *state = extend(obs, VectorConstructor(time));
  
  return true;
}
