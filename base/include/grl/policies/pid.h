/** \file pid.h
 * \brief PID policy header file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-02-15
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

#ifndef GRL_PID_POLICY_H_
#define GRL_PID_POLICY_H_

#include <grl/policies/parameterized.h>
#include <grl/policies/feed_forward.h>

namespace grl
{

/// PID policy
class PIDPolicy : public ParameterizedPolicy
{
  public:
    TYPEINFO("mapping/policy/parameterized/pid", "Parameterized policy based on a proportional-integral-derivative controller")

  protected:
    Vector setpoint_;
    size_t outputs_;
    
    LargeVector p_, i_, d_, il_, ival_;
    LargeVector params_;
    
    Vector prev_in_;
    Vector action_min_, action_max_;

  public:
    PIDPolicy() : outputs_(1) { }
  
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Policy
    virtual void act(const Observation &in, Action *out) const;
    virtual void act(double time, const Observation &in, Action *out);
    
    // From ParameterizedPolicy
    virtual size_t size() const { return params_.size(); }
    virtual const LargeVector &params() const { return params_; }
    virtual LargeVector &params() { return params_; }
};

/// PID trajectory policy
class PIDTrajectoryPolicy : public ParameterizedPolicy
{
  public:
    TYPEINFO("mapping/policy/parameterized/pidt", "Parameterized policy based on a proportional-integral-derivative controller for trajectory tracking")

  protected:
    Mapping *trajectory_;
    Vector setpoint_;     // the dynamic setpoint is taken from trajectory at the current time
    size_t inputs_, outputs_;
    LargeVector p_, i_, d_, il_, ival_;
    LargeVector params_;
    Vector prev_in_;
    Vector action_min_, action_max_;

  public:
    PIDTrajectoryPolicy() : trajectory_(NULL), inputs_(1), outputs_(1) { }

    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Policy
    virtual void act(double time, const Observation &in, Action *out);

    // From ParameterizedPolicy
    virtual size_t size() const { return params_.size(); }
    virtual const LargeVector &params() const { return params_; }
    virtual LargeVector &params() { return params_; }
};

}

#endif /* GRL_PID_POLICY_H_ */
