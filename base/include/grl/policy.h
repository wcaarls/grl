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
    virtual void act(const Vector &in, Vector *out) const = 0;
};

/// Maps states to a discrete set of actions.
class DiscretePolicy : public Policy
{
  public:
    virtual void distribution(const Vector &in, Vector *out) const = 0;
};

}

#endif /* GRL_POLICY_H_ */
