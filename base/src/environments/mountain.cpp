/** \file mountain.cpp
 * \brief Mountain world environment source file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2017-01-10
 *
 * \copyright \verbatim
 * Copyright (c) 2017, Wouter Caarls
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

#include <grl/environments/mountain.h>

using namespace grl;

REGISTER_CONFIGURABLE(MountainDynamics)
REGISTER_CONFIGURABLE(MountainRegulatorTask)

void MountainDynamics::request(ConfigurationRequest *config)
{
  config->push_back(CRP("mass", "Car mass", mass_, CRP::Configuration, 0., 100.));
  config->push_back(CRP("map", "mapping/puddle", "Height map", map_));
}

void MountainDynamics::configure(Configuration &config)
{
  map_ = (Mapping*)config["map"].ptr();
  mass_ = config["mass"];
}

void MountainDynamics::reconfigure(const Configuration &config)
{
}

void MountainDynamics::slope(const Vector &pos, Vector *angle) const
{
  Vector h;
  angle->resize(2);
  
  for (size_t ii=0; ii < 2; ++ii)
  {
    Vector posd = pos;
    posd[ii] -= 0.01;
    double height1 = map_->read(posd, &h);
    
    posd[ii] += 0.02;
    double height2 = map_->read(posd, &h);
    
    posd[ii] -= 0.01;
    (*angle)[ii] = atan2(height2-height1, 0.02);
  }
}

void MountainDynamics::eom(const Vector &state, const Vector &actuation, Vector *xd) const
{
  if (state.size() != 5 || actuation.size() != 2)
  {
    ERROR("Received state/actuation size " << state.size() << "/" << actuation.size() << ", expected 5/2");
    throw Exception("dynamics/mountain requires a task/mountain subclass");
  }

  const double g = 1, mu = 0.05;
  Vector d, pos, angle;
  
  slope(state.head(2), &angle);
  
  xd->resize(5);
  
  // Velocity/acceleration is defined along ground
  (*xd)[0] = cos(angle[0])*state[2];
  (*xd)[1] = cos(angle[1])*state[3];
  (*xd)[2] = actuation[0]/mass_ - g*sin(angle[0]) - mu*state[2];
  (*xd)[3] = actuation[1]/mass_ - g*sin(angle[1]) - mu*state[3];
  (*xd)[4] = 1;
  
  // Don't fall off the world
  // NOTE: Cannot reset position or velocity here, so just keep them from getting larger
  if (state[0] > 1)
  {
    if ((*xd)[0] > 0) (*xd)[0] = 0;
    if ((*xd)[2] > 0) (*xd)[2] = 0;
  }
  if (state[0] < 0)
  {
    if ((*xd)[0] < 0) (*xd)[0] = 0;
    if ((*xd)[2] < 0) (*xd)[2] = 0;
  }
  if (state[1] > 1)
  {
    if ((*xd)[1] > 0) (*xd)[1] = 0;
    if ((*xd)[3] > 0) (*xd)[3] = 0;
  }
  if (state[1] < 0)
  {
    if ((*xd)[1] < 0) (*xd)[1] = 0;
    if ((*xd)[3] < 0) (*xd)[3] = 0;
  }
}

void MountainRegulatorTask::configure(Configuration &config)
{
  RegulatorTask::configure(config);
  
  if (q_.size() != 4)
    throw bad_param("task/mountain/regulator:q");
  if (r_.size() != 2)
    throw bad_param("task/mountain/regulator:r");

  config.set("observation_min", VectorConstructor(0., 0., -2., -2.));
  config.set("observation_max", VectorConstructor(1., 1.,  2.,  2.));
  config.set("action_min", VectorConstructor(-1, -1));
  config.set("action_max", VectorConstructor( 1, 1));
}

void MountainRegulatorTask::observe(const Vector &state, Observation *obs, int *terminal) const
{
  if (state.size() != 5)
  {
    ERROR("Received state size " << state.size() << ", expected 5");
    throw Exception("task/mountain/regulator requires dynamics/mountain");
  }
    
  obs->v.resize(4);
  for (size_t ii=0; ii < 4; ++ii)
    (*obs)[ii] = state[ii];
  obs->absorbing = false;

  *terminal = state[4] > 20;
}

bool MountainRegulatorTask::invert(const Observation &obs, Vector *state) const
{
  *state = extend(obs, VectorConstructor(0.));
  
  return true;
}
