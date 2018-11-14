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
#include <grl/experiment.h>
#include <grl/agent.h>
#include <grl/environment.h>

namespace py = pybind11;
using namespace grl;

DECLARE_TYPE_NAME(Experiment)
DECLARE_TYPE_NAME(Environment)
DECLARE_TYPE_NAME(Agent)
DECLARE_TYPE_NAME(Mapping)

#define PYBIND11_CONFIGURABLE(x) \
  py::class_<x, Configurable, std::unique_ptr<x, py::nodelete>>(m, #x) \
    .def(py::init(&create<x>), py::keep_alive<1, 2>())


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
  
  m.def("verbosity", [](const unsigned char &v) {
      grl_log_verbosity__ = v;
    });

  // Configurator
  py::class_<Configurator>(m, "Configurator")
    .def(py::init([](const std::string &file) {
        return loadYAML(file);
      }))
    .def("__getitem__", [](Configurator &conf, const std::string &str) {
        return conf.find(str);
      }, py::return_value_policy::reference_internal)
    .def("__str__", [](Configurator &conf) { return conf.str(); })
    .def("instantiate", [](Configurator &conf) {
        return conf.instantiate();
      });
    
  // Configurable
  py::class_<Configurable, std::unique_ptr<Configurable, py::nodelete>>(m, "Configurable")
    .def("reconfigure", [](Configurable &conf, const py::dict dict) {
        Configuration mycfg;
        for (auto item : dict)
        mycfg.set(item.first.attr("__str__")().cast<std::string>(), item.second.attr("__str__")().cast<std::string>());
  
        conf.reconfigure(mycfg);
      });

  // Experiment
  PYBIND11_CONFIGURABLE(Experiment)
    .def("run", [](Experiment &exp) {
        exp.run();
      });
  
  // Environment
  PYBIND11_CONFIGURABLE(Environment)
    .def("start", [](Environment &env, int test) {
        Observation obs;
        env.start(test, &obs);
        return obs.v;
      })
    .def("step", [](Environment &env, Vector &action) {
        Observation obs;
        double reward;
        int terminal;
        env.step(action, &obs, &reward, &terminal);
        return py::make_tuple(obs.v, reward, terminal);
      });
  
  // Agent
  PYBIND11_CONFIGURABLE(Agent)
    .def("start", [](Agent &agent, Vector &obs) {
        Action action;
        agent.start(obs, &action);
        return action.v;
      })
    .def("step", [](Agent &agent, Vector &obs, double reward) {
        Action action;
        agent.step(0, obs, reward, &action);
        return action.v;
      })
    .def("end", [](Agent &agent, Vector &obs, double reward) {
        agent.end(0, obs, reward);
      });
    
  // Mapping
  PYBIND11_CONFIGURABLE(Mapping)
    .def("read", [](Mapping &mapping, Vector &in) {
        Vector out;
        mapping.read(in, &out);
        return out;
      });
}
