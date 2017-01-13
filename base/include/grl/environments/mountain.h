/** \file mountain.h
 * \brief Mountain world environment header file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2017-01-13
 *
 * \copyright \verbatim
 * Copyright (c) 2017, Wouter Caarls
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
 
#ifndef GRL_MOUNTAIN_ENVIRONMENT_H_
#define GRL_MOUNTAIN_ENVIRONMENT_H_

#include <grl/environment.h>

namespace grl
{

/// Mountain world dynamics
class MountainDynamics : public Dynamics
{
  public:
    TYPEINFO("dynamics/mountain", "Mountain world dynamics")

  protected:
    Mapping *map_;
    double mass_;
  
  public:
    MountainDynamics() : map_(NULL), mass_(1) { }
  
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Dynamics
    virtual void eom(const Vector &state, const Vector &actuation, Vector *xd) const;
    
  protected:
    void slope(const Vector &pos, Vector *angle) const;
};

/// Regulator task with possible additional penalties for entering mountains
class MountainRegulatorTask : public RegulatorTask
{
  public:
    TYPEINFO("task/mountain/regulator", "Mountain world regulator task")
    
  public:
    MountainRegulatorTask()
    {
      start_ = VectorConstructor(0.1, 0.1, 0., 0.);
      goal_ = VectorConstructor(0.9, 0.9, 0., 0.);
      stddev_ = VectorConstructor(0.1, 0.1, 0., 0.);
      q_ = VectorConstructor(1., 1., 0., 0.);
      r_ = VectorConstructor(0.01, 0.01);
    }
  
    // From Configurable
    virtual void configure(Configuration &config);

    // From Task
    virtual void observe(const Vector &state, Observation *obs, int *terminal) const;
    virtual bool invert(const Observation &obs, Vector *state) const;
};

}

#endif /* GRL_MOUNTAIN_ENVIRONMENT_H_ */
