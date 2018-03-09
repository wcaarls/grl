/** \file policy.h
 * \brief Generic policy definition.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-01-22
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

#ifndef GRL_POLICY_H_
#define GRL_POLICY_H_

#include <grl/grl.h>
#include <grl/configurable.h>
#include <grl/mapping.h>

namespace grl
{

/// Maps states to actions.
class Policy : public Mapping
{
  public:
    virtual ~Policy() { }
    
    /**
     * \brief Returns an action based on the current state.
     *
     * Called by visualizations.
     */
    virtual void act(const Observation &in, Action *out) const
    {
      throw Exception("Policy does not support visualization");
    }
    
    /**
     * \brief Returns an action based on the current time and state.
     *
     * Called by agents, once per timestep. time is 0. at the start of a new episode.
     * \note out is an inout parameter, the input being a previous or suggested action.
     */
    virtual void act(double time, const Observation &in, Action *out)
    {
      return act(in, out);
    }
    
    /// Returns the probability of taking a discrete policy's actions in a certain state.
    virtual void distribution(const Observation &in, const Action &prev, LargeVector *out) const
    {
      throw Exception("Policy does not support reading action distribution");
    }
    
    // From Mapping
    virtual double read(const Vector &in, Vector *result) const
    {
      Action action;
      act(in, &action);
      *result = action.v;
      
      if (result->size())
        return (*result)[0];
      else
        return 0;
    }
};

/// A policy based on Q or V values.
class ValuePolicy : public Policy
{
  public:
    /// Returns the expected value of the action taken in state 'in'
    virtual double value(const Observation &in) const = 0;
};

/// A parameterized Policy.
class ParameterizedPolicy : public Policy
{
  public:
    /// Returns number of policy parameters.
    virtual size_t size() const = 0;
    
    /// Returns constant policy parameter vector.
    virtual const LargeVector &params() const = 0;
    
    /// Sets policy parameter vector.
    virtual void setParams(const LargeVector &params) = 0;
};

}

#endif /* GRL_POLICY_H_ */
