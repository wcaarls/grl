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

enum TransitionType { ttUndefined, ttExploratory, ttGreedy };

/// Basic (s, a, r, s', a') state transition.
struct Transition
{
  Vector prev_obs;
  Vector prev_action;
  double reward;
  Vector obs;    ///< Empty observation signifies a terminal absorbing state with discontinued dynamics.
  Vector action; ///< Empty action signifies a terminal absorbing state with continued dynamics.
  TransitionType tt;
  
  Transition(Vector _prev_obs=Vector(), Vector _prev_action=Vector(), double _reward=0., Vector _obs=Vector(), Vector _action=Vector(), TransitionType _tt = ttUndefined) :
    prev_obs(_prev_obs), prev_action(_prev_action), reward(_reward), obs(_obs), action(_action), tt(_tt)
    {
    }
};

inline std::ostream &operator<<(std::ostream& os, const Transition& t)
{
  os << "{" << t.prev_obs << ", " << t.prev_action << "} - " << t.reward << " -> {" << t.obs << ", " << t.action << "}";
  return os;
}

/// Get path to libgrl.so.
std::string getLibraryPath();

/// Load addons.
void loadPlugins();

extern class Configurator *configurator_;
extern struct sigaction act_, oldact_;

void reconfigure();
void iSIGINT();
void iSIGSEGV();
void hSIGINT(int sig);
void hSIGSEGV(int sig);

}

#endif /* GRL_H_ */
