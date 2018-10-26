/** \file python.cpp
 * \brief Python interface source file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2018-10-22
 *
 * \copyright \verbatim
 * Copyright (c) 2018, Wouter Caarls
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

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>

#include <grl/configurable.h>
#include <grl/agent.h>
#include <grl/environment.h>

namespace py = pybind11;
using namespace grl;

DECLARE_TYPE_NAME(Environment)
DECLARE_TYPE_NAME(Agent)

template<class T>
T* create(Configurator &conf)
{
  Configurable *obj = conf.ptr();
  if (!obj)
  {
    ERROR("Configurator does not specify an instantiated object");
    return nullptr;
  }
  
  T *tobj = dynamic_cast<T*>(obj);
  if (!tobj)
  {
    ERROR("Configurator does not specify a " << type_name<T>::name);
    return nullptr;
  }
  
  return tobj;
}

PYBIND11_MODULE(grlpy, m) {
  loadPlugins();

  // Configurator
  auto c = py::class_<Configurator>(m, "Configurator");
  c.def(py::init([](const std::string &file) {
      return loadYAML(file);
    }));
  c.def("__getitem__", [](Configurator &conf, const std::string &str) {
      return conf.find(str);
    }, py::return_value_policy::reference_internal);
  c.def("__str__", [](Configurator &conf) { return conf.str(); });
  c.def("instantiate", [](Configurator &conf) {
      return conf.instantiate();
    }, py::return_value_policy::take_ownership, py::keep_alive<0, 1>());
  
  // Environment
  auto e = py::class_<Environment>(m, "Environment");
  e.def(py::init(&create<Environment>), py::return_value_policy::reference, py::keep_alive<1, 2>());
  e.def("start", [](Environment &env, int test) {
      Observation obs;
      env.start(test, &obs);
      return obs.v;
    });
  e.def("step", [](Environment &env, Vector &action) {
      Observation obs;
      double reward;
      int terminal;
      env.step(action, &obs, &reward, &terminal);
      return py::make_tuple(obs.v, reward, terminal);
    });
  
  // Agent
  auto a = py::class_<Agent>(m, "Agent");
  a.def(py::init(&create<Agent>), py::return_value_policy::reference, py::keep_alive<1, 2>());
  a.def("start", [](Agent &agent, Vector &obs) {
      Action action;
      agent.start(obs, &action);
      return action.v;
    });
  a.def("step", [](Agent &agent, Vector &obs, double reward) {
      Action action;
      agent.step(0, obs, reward, &action);
      return action.v;
    });
  a.def("end", [](Agent &agent, Vector &obs, double reward) {
      agent.end(0, obs, reward);
    });
}
