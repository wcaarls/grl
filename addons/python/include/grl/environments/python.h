/** \file python.h
 * \brief Python-based environment header file.
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
 
#ifndef GRL_PYTHON_ENVIRONMENT_H_
#define GRL_PYTHON_ENVIRONMENT_H_

#include <grl/environment.h>
#include <Python.h>

namespace grl
{

class GymEnvironment : public Environment
{
  public:
    TYPEINFO("environment/gym", "OpenAI Gym environment")

  protected:
    std::string id_;
    int interval_;
    Exporter *exporter_;
    
    PyObject *reset_, *step_, *render_;
    double time_learn_, time_test_;
    bool test_;
    size_t ep_;
  
  public:
    GymEnvironment() : id_("Pendulum-v0"), interval_(100), exporter_(NULL), reset_(NULL), step_(NULL), render_(NULL), time_learn_(0.), time_test_(0.), test_(false), ep_(0) { }
    
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Environment
    virtual void start(int test, Observation *obs);
    virtual double step(const Action &action, Observation *obs, double *reward, int *terminal);
    
  protected:
    void PyObjectToObservation(PyObject *obj, Observation *obs) const;
};

}

#endif // GRL_PYTHON_ENVIRONMENT_H_
