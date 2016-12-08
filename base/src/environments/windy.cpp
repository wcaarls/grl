/** \file windy.cpp
 * \brief Windy gridworld environment source file.
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

#include <grl/environments/windy.h>

#define EPS 0.001

using namespace grl;

REGISTER_CONFIGURABLE(WindyGridworldModel)
REGISTER_CONFIGURABLE(WindyGridworldMovementTask)

void WindyGridworldModel::request(ConfigurationRequest *config)
{
}

void WindyGridworldModel::configure(Configuration &config)
{
}

void WindyGridworldModel::reconfigure(const Configuration &config)
{
}

double WindyGridworldModel::step(const Vector &state, const Vector &action, Vector *next) const
{
  int a = action[0];
  *next = state;
  
  switch (a)
  {
    case 0: // Up
      (*next)[siY] += 1;
      break;
    case 1: // Down
      (*next)[siY] -= 1;
      break;
    case 2: // Right
      (*next)[siX] += 1;
      break;
    case 3: // Left
      (*next)[siX] -= 1;
      break;
    default:
      throw Exception("model/windy requires a task/windy subclass");
  }

  (*next)[siY] += wind_[(int)state[siX]];
  
  (*next)[siX] = fmin(fmax((*next)[siX], 0), 9);
  (*next)[siY] = fmin(fmax((*next)[siY], 0), 6);

  return 1;
}

void WindyGridworldMovementTask::request(ConfigurationRequest *config)
{
  Task::request(config);
}

void WindyGridworldMovementTask::configure(Configuration &config)
{
  config.set("observation_dims", 2);
  config.set("observation_min", VectorConstructor(0, 0));
  config.set("observation_max", VectorConstructor(9, 6));
  config.set("action_dims", 1);
  config.set("action_min", VectorConstructor(0));
  config.set("action_max", VectorConstructor(3));
  config.set("reward_min", -1);
  config.set("reward_max", 0);
}

void WindyGridworldMovementTask::reconfigure(const Configuration &config)
{
}

void WindyGridworldMovementTask::start(int test, Vector *state) const
{
  *state = VectorConstructor(0., 3., 0.);
}

void WindyGridworldMovementTask::observe(const Vector &state, Vector *obs, int *terminal) const
{
  if (state.size() != 3)
    throw Exception("task/windy/movement requires model/windy");
    
  obs->resize(2);
  for (size_t ii=0; ii < 2; ++ii)
    (*obs)[ii] = state[ii];
    
  if (succeeded(state))
    *terminal = 2;
  else
    *terminal = 0;
}

void WindyGridworldMovementTask::evaluate(const Vector &state, const Vector &action, const Vector &next, double *reward) const
{
  if (state.size() != 3 || action.size() != 1 || next.size() != 3)
    throw Exception("task/windy/movement requires model/windy");

  *reward = -1;
}

bool WindyGridworldMovementTask::invert(const Vector &obs, Vector *state) const
{
  state->resize(3);
  for (size_t ii=0; ii < 2; ++ii)
    (*state)[ii] = obs[ii];
  (*state)[WindyGridworldModel::siTime] = 0;
  
  return true;
}

bool WindyGridworldMovementTask::succeeded(const Vector &state) const
{
  if (fabs(state[WindyGridworldModel::siX] - 7) < EPS && fabs(state[WindyGridworldModel::siY] - 3) < EPS)
    return true;
  else
    return false;
}
