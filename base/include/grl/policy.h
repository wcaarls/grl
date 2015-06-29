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
     * Called at the beginning of an episode and by visualizations.
     */
    virtual void act(const Vector &in, Vector *out) const
    {
      act(Vector(), Vector(), in, out);
    }
    
    /**
     * \brief Returns an action based on the last state transition.
     *
     * Called in subsequent steps.
     */
    virtual void act(const Vector &prev_in, const Vector &prev_out, const Vector &in, Vector *out) const
    {
      act(in, out);
    }
};

/// Maps states to a discrete set of actions.
class DiscretePolicy : public Policy
{
  public:
    virtual DiscretePolicy *clone() const = 0;
    
    /// Returns action probability distribution based on the current state.
    virtual void distribution(const Vector &in, Vector *out) const
    {
      distribution(Vector(), Vector(), in, out);
    }
    
    /// Returns action probability distribution based on the last state transition.
    virtual void distribution(const Vector &prev_in, const Vector &prev_out, const Vector &in, Vector *out) const
    {
      distribution(in, out);
    }
};

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
