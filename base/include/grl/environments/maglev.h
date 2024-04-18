/** \file maglev.h
 * \brief Magnetic levitation environment header file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2024-04-18
 *
 * \copyright \verbatim
 * Copyright (c) 2024, Wouter Caarls
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
 
#ifndef GRL_MAGLEV_ENVIRONMENT_H_
#define GRL_MAGLEV_ENVIRONMENT_H_

#include <grl/environment.h>

namespace grl
{

/// 2D flyer dynamics.
class MagLevDynamics : public Dynamics
{
  public:
    TYPEINFO("dynamics/maglev", "Magnetic levitation dynamics")

  public:
    double g_, M_, R_, x_inf_, L_inf_, xi_;
    int subbu_;
  
  public:
    MagLevDynamics() : subbu_(0) { }
  
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Dynamics
    virtual void eom(const Vector &state, const Vector &actuation, Vector *xd) const;
};

/// Magnetic levitation balancing task with square root reward
class MagLevBalancingTask : public Task
{
  public:
    TYPEINFO("task/maglev/balancing", "Magnetic levitation balancing task")
    
  public:
    MagLevBalancingTask() { }
  
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Task
    virtual void start(int test, Vector *state);
    virtual void evaluate(const Vector &state, const Action &action, const Vector &next, double *reward) const;
    virtual void observe(const Vector &state, Observation *obs, int *terminal) const;
    virtual bool invert(const Observation &obs, Vector *state, double time=0.) const;
};

}

#endif /* GRL_MAGLEV_ENVIRONMENT_H_ */
