/** \file grl.h
 * \brief Main GRL header file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-01-22
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

#ifndef GRL_H_
#define GRL_H_

#define _USE_MATH_DEFINES
#include <math.h>

#include <vector>
#include <assert.h>

#include <grl/vector.h>

namespace grl
{

/// Basic (s, a, r, s', a') state transition.
struct Transition
{
  Vector prev_obs;
  Vector prev_action;
  double reward;
  Vector obs;
  Vector action;
  
  Transition(Vector _prev_obs, Vector _prev_action, double _reward, Vector _obs=Vector(), Vector _action=Vector()) :
    prev_obs(_prev_obs), prev_action(_prev_action), reward(_reward), obs(_obs), action(_action)
    {
    }
  Transition()
    {
    }
};

inline std::ostream &operator<<(std::ostream& os, const Transition& t)
{
  os << "{" << t.prev_obs << ", " << t.prev_action << "} - " << t.reward << " -> {" << t.obs << ", " << t.action << "}";
  return os;
}

/// Load addons.
void loadPlugins();

}

#endif /* GRL_H_ */
