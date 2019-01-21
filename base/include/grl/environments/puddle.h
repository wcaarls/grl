/** \file puddle.h
 * \brief Puddle world environment header file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2017-01-10
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
 
#ifndef GRL_PUDDLE_ENVIRONMENT_H_
#define GRL_PUDDLE_ENVIRONMENT_H_

#include <grl/environment.h>

namespace grl
{

/// Puddle world dynamics
class PuddleModel : public Model
{
  public:
    TYPEINFO("model/puddle", "Puddle world model")

  protected:
    double drag_;
    Mapping *map_;
  
  public:
    PuddleModel() : drag_(1.), map_(NULL) { }
  
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Model
    virtual double step(const Vector &state, const Vector &action, Vector *next) const;
};

/// Regulator task with possible additional penalties for entering puddles
class PuddleRegulatorTask : public RegulatorTask
{
  public:
    TYPEINFO("task/puddle/regulator", "Puddle world regulator task")
    
  protected:
    double penalty_;
    Mapping *map_;

  public:
    PuddleRegulatorTask() : penalty_(1.), map_(NULL)
    {
      start_ = VectorConstructor(0.1, 0.1, 0., 0.);
      goal_ = VectorConstructor(0.9, 0.9, 0., 0.);
      stddev_ = VectorConstructor(0.1, 0.1, 0., 0.);
      q_ = VectorConstructor(1., 1., 0., 0.);
      r_ = VectorConstructor(0.01, 0.01);
      timeout_ = 20;
    }
  
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Task
    virtual void observe(const Vector &state, Observation *obs, int *terminal) const;
    virtual void evaluate(const Vector &state, const Action &action, const Vector &next, double *reward) const;
    virtual bool invert(const Observation &obs, Vector *state, double time=0.) const;
};

}

#endif /* GRL_PUDDLE_ENVIRONMENT_H_ */
