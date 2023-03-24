/** \file wmr.h
 * \brief Wheeled mobile robot environment header file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2023-03-13
 *
 * \copyright \verbatim
 * Copyright (c) 2023, Wouter Caarls
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
 
#ifndef GRL_WMR_ENVIRONMENT_H_
#define GRL_WMR_ENVIRONMENT_H_

#include <grl/environment.h>

namespace grl
{

/// Wheeled mobile robot kinematics.
class WMRDynamics : public Dynamics
{
  public:
    TYPEINFO("dynamics/wmr", "Wheeled mobile robot kinematics")

  public:
    double t_, r_, l_, b_;
    int caster_;
  
  public:
    WMRDynamics() : caster_(0){ }
  
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Dynamics
    virtual void eom(const Vector &state, const Vector &actuation, Vector *xd) const;
};

/// Wheeled mobile robot reaching task with quadratic costs
class WMRRegulatorTask : public RegulatorTask
{
  public:
    TYPEINFO("task/wmr/regulator", "Wheeled mobile robot regulator task")
    
  protected:
    double v_linear_, v_angular_;

  public:
    WMRRegulatorTask() : v_linear_(1.0), v_angular_(1.0)
    {
      start_ = VectorConstructor(0, 0, 0);
      goal_ = VectorConstructor(0, 0, 0);
      stddev_ = VectorConstructor(2., 2., 3.14);
      q_ = VectorConstructor(1, 1, 1);
      r_ = VectorConstructor(0.01, 0.01);
      timeout_ = 9.99;
    }
  
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Task
    virtual void evaluate(const Vector &state, const Action &action, const Vector &next, double *reward) const;
    virtual void observe(const Vector &state, Observation *obs, int *terminal) const;
    virtual bool invert(const Observation &obs, Vector *state, double time=0.) const;
};

/// Wheeled mobile robot reaching task with quadratic costs that includes caster wheels
class WMRCasterRegulatorTask : public RegulatorTask
{
  public:
    TYPEINFO("task/wmr/caster_regulator", "Wheeled mobile robot regulator task with casters")
    
  protected:
    double v_linear_, v_angular_;

  public:
    WMRCasterRegulatorTask() : v_linear_(1.0), v_angular_(1.0)
    {
      start_ = VectorConstructor(0, 0, 0, 0, 0);
      goal_ = VectorConstructor(0, 0, 0, 0, 0);
      stddev_ = VectorConstructor(2., 2., 3.14, 3.14, 3.14);
      q_ = VectorConstructor(1, 1, 1, 1, 1);
      r_ = VectorConstructor(0.01, 0.01);
      timeout_ = 9.99;
    }
  
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Task
    virtual void evaluate(const Vector &state, const Action &action, const Vector &next, double *reward) const;
    virtual void observe(const Vector &state, Observation *obs, int *terminal) const;
    virtual bool invert(const Observation &obs, Vector *state, double time=0.) const;
};

}

#endif /* GRL_WMR_ENVIRONMENT_H_ */
