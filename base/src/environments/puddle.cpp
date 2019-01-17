/** \file puddle.cpp
 * \brief Puddle world environment source file.
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

#include <grl/environments/puddle.h>

using namespace grl;

REGISTER_CONFIGURABLE(PuddleModel)
REGISTER_CONFIGURABLE(PuddleRegulatorTask)

void PuddleModel::request(ConfigurationRequest *config)
{
  config->push_back(CRP("drag", "Velocity multiplier for puddles", drag_));
  config->push_back(CRP("map", "mapping/puddle", "Puddle map", map_));
}

void PuddleModel::configure(Configuration &config)
{
  drag_ = config["drag"];
  map_ = (Mapping*)config["map"].ptr();
}

void PuddleModel::reconfigure(const Configuration &config)
{
}

double PuddleModel::step(const Vector &state, const Vector &actuation, Vector *next) const
{
  if (state.size() != 5 || actuation.size() != 2)
    throw Exception("model/puddle requires a task/puddle subclass");
    
  Vector d, n = state;
  double depth = map_->read(VectorConstructor(state[0], state[1]), &d);
  
  double h = 0.01;
  double drag = pow(drag_ + (1-drag_)*depth, 1./5);
  
  for (size_t ii=0; ii < 5; ++ii)
  {
    // Euler
    n[0] =  n[0] + h*n[2];
    n[1] =  n[1] + h*n[3];
    n[2] = (n[2] + h*actuation[0])*drag;
    n[3] = (n[3] + h*actuation[1])*drag;
    n[4] =  n[4] + h;

    // Bounce off edges
    if (n[0] < 0 && n[2] < 0) n[2] = -0.5*n[2];
    if (n[0] > 1 && n[2] > 0) n[2] = -0.5*n[2];
    if (n[1] < 0 && n[3] < 0) n[3] = -0.5*n[3];
    if (n[1] > 1 && n[3] > 0) n[3] = -0.5*n[3];
  }
  
  *next = n;
  
  return 0.05;
}

void PuddleRegulatorTask::request(ConfigurationRequest *config)
{
  RegulatorTask::request(config);

  config->push_back(CRP("penalty", "Penalty multiplier for puddles", penalty_, CRP::Configuration, 0., DBL_MAX));
  config->push_back(CRP("map", "mapping/puddle", "Puddle map", map_, CRP::System));
}

void PuddleRegulatorTask::configure(Configuration &config)
{
  RegulatorTask::configure(config);
  
  penalty_ = config["penalty"];
  map_ = (Mapping*) config["map"].ptr();
  
  if (q_.size() != 4)
    throw bad_param("task/puddle/regulator:q");
  if (r_.size() != 2)
    throw bad_param("task/puddle/regulator:r");

  config.set("observation_min", VectorConstructor(0., 0., -2., -2.));
  config.set("observation_max", VectorConstructor(1., 1.,  2.,  2.));
  config.set("action_min", VectorConstructor(-1, -1));
  config.set("action_max", VectorConstructor( 1, 1));
}

void PuddleRegulatorTask::reconfigure(const Configuration &config)
{
  RegulatorTask::reconfigure(config);
}

void PuddleRegulatorTask::observe(const Vector &state, Observation *obs, int *terminal) const
{
  if (state.size() != 5)
    throw Exception("task/puddle/regulator requires dynamics/puddle");
    
  obs->v.resize(4);
  for (size_t ii=0; ii < 4; ++ii)
    (*obs)[ii] = state[ii];
  obs->absorbing = false;

  *terminal = state[4] > 20;
}

void PuddleRegulatorTask::evaluate(const Vector &state, const Action &action, const Vector &next, double *reward) const
{
  RegulatorTask::evaluate(state, action, next, reward);

  Vector depth;  
  map_->read(VectorConstructor(state[0], state[1]), &depth);
  
  *reward -= penalty_*depth[0];
}

bool PuddleRegulatorTask::invert(const Observation &obs, Vector *state, double time) const
{
  *state = extend(obs, VectorConstructor(time));
  
  return true;
}
