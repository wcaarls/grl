/** \file tennessee.h
 * \brief Tennessee Eastman process environment header file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2020-05-17
 *
 * \copyright \verbatim
 * Copyright (c) 2020, Wouter Caarls
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
 
#ifndef GRL_TENNESSEE_EASTMAN_ENVIRONMENT_H_
#define GRL_TENNESSEE_EASTMAN_ENVIRONMENT_H_

#include <grl/environment.h>
#include <grl/policy.h>

namespace grl
{

/// Tennessee Eastman process dynamics.
class TennesseeEastmanDynamics : public Dynamics
{
  public:
    TYPEINFO("dynamics/tennessee", "Tennessee Eastman process dynamics")
    
  public:
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Dynamics
    virtual void eom(const Vector &state, const Vector &actuation, Vector *xd) const;
};

/// Generic Tennessee Eastman task.
class TennesseeEastmanTask : public Task
{
  public:
    double T_, randomization_;
    double tau_, action_tau_;
    std::string control_;
    Vector observation_idx_, action_idx_;
    double terminal_penalty_;
  
  public:
    TennesseeEastmanTask() : T_(7200), randomization_(0.), tau_(1.), action_tau_(0.), control_("setpoint"), terminal_penalty_(1000) { }
  
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Task
    virtual void start(int test, Vector *state);
    virtual bool actuate(const Vector &prev, const Vector &state, const Action &action, Vector *actuation) const;
    virtual void observe(const Vector &state, Observation *obs, int *terminal) const;
    virtual void evaluate(const Vector &state, const Action &action, const Vector &next, double *reward) const;
    
    virtual double calculateReward(const double *XMEAS, const Action &action) const = 0;
};

/// Tennessee Eastman regulation task.
class TennesseeEastmanRegulationTask : public TennesseeEastmanTask
{
  public:
    TYPEINFO("task/tennessee/regulation", "Tennessee Eastman regulation task")
  
  public:
    // From TennesseeEastmanTask
    virtual double calculateReward(const double *XMEAS, const Action &action) const;
};

}

#endif /* GRL_TENNESSEE_EASTMAN_ENVIRONMENT_H_ */
