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

#include <grl/configurable.h>
#include <grl/grl.h>

namespace grl
{

/// Maps states to actions.
class Policy : public Configurable
{
  public:
    virtual ~Policy() { }
    virtual Policy *clone() const = 0;
    
    /**
     * \brief Returns an action based on the current state.
     *
     * Called by visualizations.
     */
    virtual TransitionType act(const Vector &in, Vector *out) const
    {
      throw Exception("Policy does not support visualization");
      return ttUndefined;
    }
    
    /**
     * \brief Returns an action based on the current time and state.
     *
     * Called by agents, once per timestep. time is 0. at the start of a new episode.
     * \note out is an inout parameter, the input being a previous or suggested action.
     */
    virtual TransitionType act(double time, const Vector &in, Vector *out)
    {
      return act(in, out);
    }
};

/// A parameterized Policy.
class ParameterizedPolicy : public Policy
{
  public:
    virtual ParameterizedPolicy *clone() const = 0;
    
    /// Returns number of policy parameters.
    virtual size_t size() const = 0;
    
    /// Returns constant policy parameter vector.
    virtual const Vector &params() const = 0;
    
    /// Returns policy parameter vector.
    virtual Vector &params() = 0;
};

}

#endif /* GRL_POLICY_H_ */
