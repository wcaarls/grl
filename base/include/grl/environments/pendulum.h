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
    TYPEINFO("dynamics/pendulum")

  public:
    double J_, m_, g_, l_, b_, K_, R_;
  
  public:
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Dynamics
    virtual PendulumDynamics *clone() const;
    virtual void eom(const Vector &state, const Vector &action, Vector *xd) const;
};

/// Pendulum swing-up task.
class PendulumSwingupTask : public Task
{
  public:
    TYPEINFO("task/pendulum/swingup")
  
  public:
    double T_, randomization_;
  
  public:
    PendulumSwingupTask() : T_(2.99), randomization_(0.) { }
  
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Task
    virtual PendulumSwingupTask *clone() const;
    virtual void start(int test, Vector *state) const;
    virtual void observe(const Vector &state, Vector *obs, int *terminal) const;
    virtual void evaluate(const Vector &state, const Vector &action, const Vector &next, double *reward) const;
    virtual bool invert(const Vector &obs, Vector *state) const;
};

}

#endif /* GRL_PENDULUM_ENVIRONMENT_H_ */
