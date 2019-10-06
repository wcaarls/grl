/** \file flyer2d.cpp
 * \brief 2D flyer environment source file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2016-07-13
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

#include <grl/environments/flyer2d.h>

using namespace grl;

REGISTER_CONFIGURABLE(Flyer2DDynamics)
REGISTER_CONFIGURABLE(Flyer2DRegulatorTask)

void Flyer2DDynamics::request(ConfigurationRequest *config)
{
  config->push_back(CRP("obstacle", "Simulate obstacle below origin", obstacle_, CRP::Configuration, 0, 1));
}

void Flyer2DDynamics::configure(Configuration &config)
{
  m_ = 0.1;
  g_ = 9.81;
  l_ = 0.1;
  I_ = m_*4*l_*l_/12; // Rod
  
  obstacle_ = config["obstacle"];
}

void Flyer2DDynamics::reconfigure(const Configuration &config)
{
}

void Flyer2DDynamics::eom(const Vector &state, const Vector &actuation, Vector *xd) const
{
  if (state.size() != 7 || actuation.size() != 2)
    throw Exception("dynamics/flyer2d requires a task/flyer2d subclass");

  xd->resize(7);
  (*xd)[0] = state[3];
  (*xd)[1] = state[4];
  (*xd)[2] = state[5];
  (*xd)[3] = -(1.+actuation[0]+actuation[1])*sin(state[2])/m_;
  (*xd)[4] =  (1.+actuation[0]+actuation[1])*cos(state[2])/m_-g_;
  (*xd)[5] =  (actuation[1]-actuation[0])*l_/I_;
  (*xd)[6] = 1;

  // Simulate walls
  // NOTE: Cannot reset position or velocity here, so just keep them from getting larger
  if (state[0] > 1)
  {
    if ((*xd)[0] > 0) (*xd)[0] = 0;
    if ((*xd)[3] > 0) (*xd)[3] = 0;
  }
  if (state[0] < -1)
  {
    if ((*xd)[0] < 0) (*xd)[0] = 0;
    if ((*xd)[3] < 0) (*xd)[3] = 0;
  }
  if (state[1] > 1)
  {
    if ((*xd)[1] > 0) (*xd)[1] = 0;
    if ((*xd)[4] > 0) (*xd)[4] = 0;
  }
  if (state[1] < -1)
  {
    if ((*xd)[1] < 0) (*xd)[1] = 0;
    if ((*xd)[4] < 0) (*xd)[4] = 0;
  }
  
  if (obstacle_)
  {
    // Simulate obstacle
    if (state[0] > -0.4 && state[0] < 0.1)
    {
      if (state[1] > -0.3 && state[1] < -0.2)
      {
        // Cannot move up through obstacle, but can move down
        if ((*xd)[1] > 0)
        {
          (*xd)[1] = 0;
          if ((*xd)[4] > 0) (*xd)[4] = 0;
        }
      }
    }
  }
}

// Regulator

void Flyer2DRegulatorTask::request(ConfigurationRequest *config)
{
  RegulatorTask::request(config);
  
  config->push_back(CRP("action_range", "Range of allowed actions", action_range_, CRP::Configuration, 0., 1.));
}

void Flyer2DRegulatorTask::configure(Configuration &config)
{
  RegulatorTask::configure(config);
  
  action_range_ = config["action_range"];
  
  if (q_.size() != 6)
    throw bad_param("task/flyer2d/regulator:q");
  if (r_.size() != 2)
    throw bad_param("task/flyer2d/regulator:r");

  config.set("observation_min", VectorConstructor(-1, -1, -M_PI, -10, -10, -10*M_PI));
  config.set("observation_max", VectorConstructor( 1,  1,  M_PI,  10,  10,  10*M_PI));
  config.set("action_min", VectorConstructor(-action_range_/2, -action_range_/2));
  config.set("action_max", VectorConstructor( action_range_/2,  action_range_/2));
}

void Flyer2DRegulatorTask::reconfigure(const Configuration &config)
{
  RegulatorTask::reconfigure(config);
}


void Flyer2DRegulatorTask::evaluate(const Vector &state, const Action &action, const Vector &next, double *reward) const
{
  // Bound angular error to pi 
  Vector _state = state, _next = next;
  _state[2] = fmod(_state[2], M_PI);
  _next[2] = fmod(_next[2], M_PI);
  
  RegulatorTask::evaluate(_state, action, _next, reward);
}

void Flyer2DRegulatorTask::observe(const Vector &state, Observation *obs, int *terminal) const
{
  RegulatorTask::observe(state, obs, terminal);

  if (state.size() != 7)
    throw Exception("task/flyer2d/regulator requires dynamics/flyer2d");
    
  obs->v.resize(6);
  for (size_t ii=0; ii < 6; ++ii)
    (*obs)[ii] = state[ii];
    
  double a = fmod(state[2]+M_PI, 2*M_PI);
  if (a < 0) a += 2*M_PI;
  a -= M_PI;
  
  (*obs)[2] = a;
    
  obs->absorbing = false;
  
  if (fabs(state[0]) >= 1 || fabs(state[1]) >= 1)
    *terminal = 1;
}

bool Flyer2DRegulatorTask::invert(const Observation &obs, Vector *state, double time) const
{
  *state = extend(obs, VectorConstructor(time));
  
  return true;
}
