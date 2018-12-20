/** \file python.cpp
 * \brief Python-based environment source file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2018-12-19
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

#include <grl/environments/python.h>

using namespace grl;

REGISTER_CONFIGURABLE(GymEnvironment)

void GymEnvironment::request(ConfigurationRequest *config)
{
  config->push_back(CRP("id", "OpenAI Gym environment Id", id_));
  config->push_back(CRP("render", "Render interval (0=no rendering)", interval_));
  config->push_back(CRP("exporter", "exporter", "Optional exporter for transition log (supports observation, action, reward, terminal)", exporter_, true));
}

void GymEnvironment::configure(Configuration &config)
{
  id_ = config["id"].str();
  interval_ = config["render"];
  exporter_ = (Exporter*)config["exporter"].ptr();
  
  // Register fields to be exported
  if (exporter_)
    exporter_->init({"observation", "action", "reward", "terminal"});
  
  Py_Initialize();
  PyObject *module = PyImport_Import(PyUnicode_DecodeFSDefault("gym"));
  
  if (!module)
  {
    PyErr_Print();
    throw Exception("Couldn't import gym module");
  }
  
  PyObject *make = PyObject_GetAttrString(module, "make");
  if (!make || !PyCallable_Check(make))
  {
    PyErr_Print();
    throw Exception("Couldn't find gym.make()");
  }
  
  PyObject *id = PyUnicode_DecodeFSDefault(id_.c_str());
  PyObject *args = PyTuple_New(1);
  PyTuple_SetItem(args, 0, id);
  PyObject *env = PyObject_CallObject(make, args);
  Py_DECREF(args);
  
  if (!env)
  {
    PyErr_Print();
    throw bad_param("environment/gym:id");
  }
  
  reset_ = PyObject_GetAttrString(env, "reset");  
  step_ = PyObject_GetAttrString(env, "step");  
  render_ = PyObject_GetAttrString(env, "render");  
  
  Py_DECREF(env);
}

void GymEnvironment::reconfigure(const Configuration &config)
{
  if (config.has("action") && config["action"].str() == "reset")
  {
    time_learn_ = time_test_ = 0.;
    ep_ = 0;
  }
}


void GymEnvironment::start(int test, Observation *obs)
{
  test_ = test;
  ep_++;

  PyObject *ret = PyObject_CallObject(reset_, NULL);
  PyObjectToObservation(ret, obs);
  Py_DECREF(ret);
  
  if (exporter_)
    exporter_->open((test_?"test":"learn"), (test_?time_test_:time_learn_) != 0.0);
}

double GymEnvironment::step(const Action &action, Observation *obs, double *reward, int *terminal)
{
  // Do step
  PyObject *args = PyTuple_New(1);
  PyObject *arg = PyTuple_New(action.size());
  for (size_t ii=0; ii != action.size(); ++ii)
    PyTuple_SetItem(arg, ii, PyFloat_FromDouble(action[ii]));
  PyTuple_SetItem(args, 0, arg);
  
  PyObject *ret = PyObject_CallObject(step_, args);
  Py_DECREF(args);
  if (PyObject_Length(ret) != 4)
  {
    PyErr_Print();
    throw Exception("Environment returned invalid response");
  }
  PyObjectToObservation(PyTuple_GetItem(ret, 0), obs);
  *reward = PyFloat_AsDouble(PyTuple_GetItem(ret, 1));
  *terminal = PyFloat_AsDouble(PyTuple_GetItem(ret, 2));
  Py_DECREF(ret);
  
  if (interval_ && !(ep_%interval_))
    PyObject_CallObject(render_, NULL);

  double &time = test_?time_test_:time_learn_;
  
  if (exporter_)
    exporter_->write({*obs, action, VectorConstructor(*reward), VectorConstructor((double)*terminal)});

  time++;
  return 1;
}

void GymEnvironment::PyObjectToObservation(PyObject *obj, Observation *obs) const
{
  if (!obj)
  {
    PyErr_Print();
    throw Exception("Python observation object doesn't exist");
  }

  int len = PyObject_Length(obj);
  if (len < 0)
  {
    PyErr_Print();
    throw Exception("Could not get Python observation length");
  }
  
  obs->v.resize(len);
  for (size_t ii=0; ii != len; ++ii)
  {
    PyObject *val = PyObject_GetItem(obj, PyLong_FromLong(ii));
    (*obs)[ii] = PyFloat_AsDouble(val);
    Py_XDECREF(val);
  }
}
