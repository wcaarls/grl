/** \file acrobot.h
 * \brief Acrobot environment header file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-08-18
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
 
#ifndef GRL_ACROBOT_ENVIRONMENT_H_
#define GRL_ACROBOT_ENVIRONMENT_H_

#include <grl/environment.h>

namespace grl
{

/// Acrobot dynamics, from Sutton&Barto, section 11.3.
class AcrobotDynamics : public Dynamics
{
  public:
    enum StateDimIndex { siAngle1, siAngle2, siAngleRate1, siAngleRate2, siTime };
    enum ActionDimIndex { aiTau };
    
    TYPEINFO("dynamics/acrobot", "Acrobot dynamics")
    
  public:
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Dynamics
    virtual void eom(const Vector &state, const Vector &actuation, Vector *xd) const;
};

/// Acrobot swing-up task.
class AcrobotBalancingTask : public Task
{
  public:
    TYPEINFO("task/acrobot/balancing", "Acrobot balancing task")
  
  public:
    AcrobotBalancingTask() { }
  
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Task
    virtual void start(int test, Vector *state);
    virtual void observe(const Vector &state, Observation *obs, int *terminal) const;
    virtual void evaluate(const Vector &state, const Action &action, const Vector &next, double *reward) const;
    virtual bool invert(const Observation &obs, Vector *state, double time=0.) const;

  protected:
    bool failed(const Vector &state) const;
};

/// Acrobot balancing task with quadratic costs
class AcrobotRegulatorTask : public RegulatorTask
{
  public:
    TYPEINFO("task/acrobot/regulator", "Acrobot regulator task")

  public:
    AcrobotRegulatorTask()
    {
      start_ = VectorConstructor(M_PI, 0, 0, 0);
      goal_ = VectorConstructor(M_PI, 0, 0, 0);
      stddev_ = VectorConstructor(0.005, 0.005, 0, 0);
      q_ = VectorConstructor(1, 1, 0, 0);
      r_ = VectorConstructor(0.01);
      timeout_ = 20;
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

#endif /* GRL_ACROBOT_ENVIRONMENT_H_ */
