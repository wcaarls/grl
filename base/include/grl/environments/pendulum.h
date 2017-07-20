/** \file pendulum.h
 * \brief Pendulum environment header file.
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
 
#ifndef GRL_PENDULUM_ENVIRONMENT_H_
#define GRL_PENDULUM_ENVIRONMENT_H_

#include <grl/environment.h>

namespace grl
{

/// Pendulum dynamics, modeled after the DCSC MOPS.
class PendulumDynamics : public Dynamics
{
  public:
    TYPEINFO("dynamics/pendulum", "Pendulum dynamics based on the DCSC MOPS")

  public:
    double J_, m_, g_, l_, b_, K_, R_;
  
  public:
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Dynamics
    virtual void eom(const Vector &state, const Vector &actuation, Vector *xd) const;
};

/// Pendulum swing-up task.
class PendulumSwingupTask : public Task
{
  public:
    TYPEINFO("task/pendulum/swingup", "Pendulum swing-up task")
  
  public:
    double T_, randomization_;
  
  public:
    PendulumSwingupTask() : T_(2.99), randomization_(0.) { }
  
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Task
    virtual void start(int test, Vector *state) const;
    virtual bool actuate(const Vector &state, const Action &action, Vector *actuation) const
    {
      *actuation = VectorConstructor(fmin(fmax(action[0], -3), 3));
      return true;
    }
    virtual void observe(const Vector &state, Observation *obs, int *terminal) const;
    virtual void evaluate(const Vector &state, const Action &action, const Vector &next, double *reward) const;
    virtual bool invert(const Observation &obs, Vector *state) const;
};

/// Pendulum balancing task with quadratic costs
class PendulumRegulatorTask : public RegulatorTask
{
  public:
    TYPEINFO("task/pendulum/regulator", "Pendulum regulator task")

  public:
    PendulumRegulatorTask()
    {
      start_ = VectorConstructor(M_PI, 0);
      goal_ = VectorConstructor(0, 0);
      stddev_ = VectorConstructor(0.1, 0);
      q_ = VectorConstructor(1, 0);
      r_ = VectorConstructor(0.01);
    }
  
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Task
    virtual void observe(const Vector &state, Observation *obs, int *terminal) const;
    virtual bool invert(const Observation &obs, Vector *state) const;
};

}

#endif /* GRL_PENDULUM_ENVIRONMENT_H_ */
