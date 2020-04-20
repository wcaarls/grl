/** \file reactor.h
 * \brief van de Vusse continuous stirred tank reactor environment header file.
 *
 * \author    Eric Luz <ercluz@gmail.com>
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2019-07-17
 *
 * \copyright \verbatim
 * Copyright (c) 2019, Wouter Caarls
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
 
#ifndef GRL_REACTOR_ENVIRONMENT_H_
#define GRL_REACTOR_ENVIRONMENT_H_

#include <grl/environment.h>

namespace grl
{

/// van de Vusse CSTR dynamics
class ReactorDynamics : public Dynamics
{
  public:
    TYPEINFO("dynamics/reactor", "Reactor dynamics based on the DCSC MOPS")

  public:
    Vector k0_, Ea_, Dh_;
    double ro_, Cp_, v_, kw_, Ar_, mk_, Cpk_, Cain_, Cbin_, Tin_, Tkf_, Vk_;
  
  public:
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Dynamics
    virtual void eom(const Vector &state, const Vector &actuation, Vector *xd) const;
};

/// Generic van de Vusse CSTR task
class ReactorTask : public Task
{
  public:
    double T_, randomization_;
  
  public:
    ReactorTask() : T_(3600.), randomization_(0.) { }
  
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Task
    virtual void start(int test, Vector *state);
    virtual bool actuate(const Vector &prev, const Vector &state, const Action &action, Vector *actuation) const;
    virtual void observe(const Vector &state, Observation *obs, int *terminal) const;
    virtual bool invert(const Observation &obs, Vector *state, double time=0.) const;
};

/// van de Vusse CSTR balancing task at Fb = Fin * Cb = setpoint
class ReactorBalancingTask : public ReactorTask
{
  public:
    TYPEINFO("task/reactor/balancing", "Maintain reactor at Fb setpoint")
  
  public:
    double setpoint_;
  
  public:
    ReactorBalancingTask() : setpoint_(0.) { }
  
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);

    // From Task
    virtual void evaluate(const Vector &state, const Action &action, const Vector &next, double *reward) const;
};

/// van de Vusse CSTR setpoint tracking task
class ReactorTrackingTask : public ReactorTask
{
  public:
    TYPEINFO("task/reactor/tracking", "Track Fb setpoint")
  
  public:
    double min_, max_;
    int setpoints_;
    Mapping *profile_;
    
    Matrix timeline_;
    int test_;
  
  public:
    ReactorTrackingTask() : min_(100), max_(600), setpoints_(3), profile_(NULL) { }
  
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);

    // From Task
    virtual void start(int test, Vector *state);
    virtual void observe(const Vector &state, Observation *obs, int *terminal) const;
    virtual void evaluate(const Vector &state, const Action &action, const Vector &next, double *reward) const;
    
  protected:
    double setpoint(double time) const;
};

/// van de Vusse CSTR maximization task
class ReactorMaximizationTask : public ReactorTask
{
  public:
    TYPEINFO("task/reactor/maximization", "Maximize Cb and Fin")
  
  public:
    double fin_weight_;
  
  public:
    ReactorMaximizationTask() : fin_weight_(0.) { }
  
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);

    // From Task
    virtual void evaluate(const Vector &state, const Action &action, const Vector &next, double *reward) const;
};

}

#endif /* GRL_REACTOR_ENVIRONMENT_H_ */
