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
}

void Flyer2DDynamics::configure(Configuration &config)
{
  m_ = 0.1;
  g_ = 9.81;
  l_ = 0.1;
  I_ = m_*4*l_*l_/12; // Rod
}

void Flyer2DDynamics::reconfigure(const Configuration &config)
{
}

void Flyer2DDynamics::eom(const Vector &state, const Vector &action, Vector *xd) const
{
  if (state.size() != 7 || action.size() != 2)
    throw Exception("dynamics/flyer2d requires a task/flyer2d subclass");

  xd->resize(7);
  (*xd)[0] = state[3];
  (*xd)[1] = state[4];
  (*xd)[2] = state[5];
  (*xd)[3] = -(action[0]+action[1])*sin(state[2])/m_;
  (*xd)[4] =  (action[0]+action[1])*cos(state[2])/m_-g_;
  (*xd)[5] =  (action[1]-action[0])*l_/I_;
  (*xd)[6] = 1;
}

// Regulator

void Flyer2DRegulatorTask::request(ConfigurationRequest *config)
{
  RegulatorTask::request(config);
}

void Flyer2DRegulatorTask::configure(Configuration &config)
{
  RegulatorTask::configure(config);
  
  if (q_.size() != 6)
    throw bad_param("task/flyer2d/regulator:q");
  if (r_.size() != 2)
    throw bad_param("task/flyer2d/regulator:r");

  config.set("observation_min", VectorConstructor(-1, -1, -M_PI, -10, -10, -10*M_PI));
  config.set("observation_max", VectorConstructor( 1,  1,  M_PI,  10,  10,  10*M_PI));
  config.set("action_min", VectorConstructor(0, 0));
  config.set("action_max", VectorConstructor(1, 1));
}

void Flyer2DRegulatorTask::reconfigure(const Configuration &config)
{
  RegulatorTask::reconfigure(config);
}

void Flyer2DRegulatorTask::observe(const Vector &state, Vector *obs, int *terminal) const
{
  if (state.size() != 7)
    throw Exception("task/flyer2d/regulator requires dynamics/flyer2d");
    
  obs->resize(6);
  for (size_t ii=0; ii < 6; ++ii)
    (*obs)[ii] = state[ii];

  *terminal = state[6] > 3;
}

bool Flyer2DRegulatorTask::invert(const Vector &obs, Vector *state) const
{
  *state = extend(obs, VectorConstructor(0.));
  
  return true;
}
