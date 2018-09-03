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

#include <getopt.h>

#include <vector>
#include <assert.h>

#include <grl/vector.h>

namespace grl
{

enum ActionType { atUndefined, atExploratory, atGreedy };

struct Observation
{
  Vector v, u;
  bool absorbing;
  
  Observation() { }
  Observation(const Vector &_v, bool _absorbing=false) : v(_v), absorbing(_absorbing) { }
  operator Vector&() { return v; }
  operator const Vector&() const { return v; }
  Observation &operator=(const Vector &_v)
  {
    v = _v;
    return *this;
  }
  
  double& operator[](int idx) { return v[idx]; }
  const double& operator[](int idx) const { return v[idx]; }
  size_t size() const { return v.size(); }
};

inline std::ostream &operator<<(std::ostream& os, const Observation& o)
{
  if (o.absorbing)
    os << "abs:[";
  else
    os << "reg:[";

  os << o.v << "]";
  return os;
}

struct Action
{
  Vector v;
  ActionType type;

  Action() { type = atUndefined; }
  Action(const Vector &_v, ActionType _type=atUndefined) : v(_v), type(_type) { }
  operator Vector&() { return v; }
  operator const Vector&() const { return v; }
  Action &operator=(const Vector &_v)
  {
    v = _v;
    type = atUndefined;
    return *this;
  }

  double& operator[](int idx) { return v[idx]; }
  const double& operator[](int idx) const { return v[idx]; }
  size_t size() const { return v.size(); }
};

inline std::ostream &operator<<(std::ostream& os, const Action& a)
{
  switch (a.type)
  {
    case atUndefined:
      os << "und:[";
      break;
    case atExploratory:
      os << "exp:[";
      break;
    case atGreedy:
      os << "gdy:[";
      break;
  }

  os << a.v << "]";
  return os;
}

/// Basic (s, a, r, s', a') state transition.
struct Transition
{
  Observation prev_obs;
  Action prev_action;
  double reward;
  Observation obs;    ///< Empty observation signifies a terminal absorbing state with discontinued dynamics.
  Action action; ///< Empty action signifies a terminal absorbing state with continued dynamics.
  
  Transition(Observation _prev_obs=Observation(), Action _prev_action=Action(), double _reward=0., Observation _obs=Observation(), Action _action=Action()) :
    prev_obs(_prev_obs), prev_action(_prev_action), reward(_reward), obs(_obs), action(_action)
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
