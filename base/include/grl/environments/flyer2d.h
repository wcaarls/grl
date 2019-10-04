/** \file flyer2d.h
 * \brief 2D flyer environment header file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2016-07-13
 *
 * \copyright \verbatim
 * Copyright (c) 2016, Wouter Caarls
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
 
#ifndef GRL_FLYER2D_ENVIRONMENT_H_
#define GRL_FLYER2D_ENVIRONMENT_H_

#include <grl/environment.h>

namespace grl
{

/// 2D flyer dynamics.
class Flyer2DDynamics : public Dynamics
{
  public:
    TYPEINFO("dynamics/flyer2d", "2D flyer dynamics")

  public:
    double m_, g_, l_, I_;
  
  public:
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Dynamics
    virtual void eom(const Vector &state, const Vector &actuation, Vector *xd) const;
};

/// 2D flyer hovering task with quadratic costs
class Flyer2DRegulatorTask : public RegulatorTask
{
  public:
    TYPEINFO("task/flyer2d/regulator", "2D flyer regulator task")
    
  protected:
    double action_range_;

  public:
    Flyer2DRegulatorTask() : action_range_(1.0)
    {
      start_ = VectorConstructor(0, 0, 0, 0, 0, 0);
      goal_ = VectorConstructor(0, 0, 0, 0, 0 ,0);
      stddev_ = VectorConstructor(0.1, 0.1, 0.1, 0, 0, 0);
      q_ = VectorConstructor(1, 1, 1, 0, 0, 0);
      r_ = VectorConstructor(0.01, 0.01);
      timeout_ = 2.99;
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

#endif /* GRL_FLYER2D_ENVIRONMENT_H_ */
