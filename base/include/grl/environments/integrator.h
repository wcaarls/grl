/** \file integrator.h
 * \brief Integrator environment header file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2018-05-18
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
 
#ifndef GRL_INTEGRATOR_ENVIRONMENT_H_
#define GRL_INTEGRATOR_ENVIRONMENT_H_

#include <grl/environment.h>

namespace grl
{

/// Integrator dynamics.
class IntegratorDynamics : public Dynamics
{
  public:
    TYPEINFO("dynamics/integrator", "Integrator dynamics")

  public:
    size_t order_;
  
  public:
    IntegratorDynamics() : order_(2) { }
  
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Dynamics
    virtual void eom(const Vector &state, const Vector &actuation, Vector *xd) const;
};

/// Integrator regulator task with quadratic costs
class IntegratorRegulatorTask : public RegulatorTask
{
  public:
    TYPEINFO("task/integrator/regulator", "Integrator regulator task")
    
  public:
    size_t order_;

  public:
    IntegratorRegulatorTask()
    {
      order_ = 2;
      start_ = VectorConstructor(1, 0);
      goal_ = VectorConstructor(0, 0);
      stddev_ = VectorConstructor(1, 0);
      q_ = VectorConstructor(1, 0);
      r_ = VectorConstructor(0.01);
      timeout_ = 3;
    }
  
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Task
    virtual void observe(const Vector &state, Observation *obs, int *terminal) const;
    virtual bool invert(const Observation &obs, Vector *state, double time=0.) const;
};

}

#endif /* GRL_INTEGRATOR_ENVIRONMENT_H_ */
