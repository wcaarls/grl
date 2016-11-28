/** \file random.h
 * \brief Discrete and continuous random policies header file.
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

#ifndef GRL_RANDOM_POLICY_H_
#define GRL_RANDOM_POLICY_H_

#include <grl/policy.h>
#include <grl/discretizer.h>

namespace grl
{

/// Continuous random policy
class RandomPolicy : public Policy
{
  public:
    TYPEINFO("policy/random", "Policy that chooses continuous random actions")

  protected:
    Vector min_, max_;

  public:
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Policy
    virtual RandomPolicy *clone() const;
    virtual TransitionType act(const Vector &in, Vector *out) const;
};

/// Discrete random policy
class RandomDiscretePolicy : public Policy
{
  public:
    TYPEINFO("policy/discrete/random", "Policy that chooses discrete random actions")

  protected:
    Discretizer *discretizer_;

  public:
    RandomDiscretePolicy() : discretizer_(NULL) { }
  
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Policy
    virtual RandomDiscretePolicy *clone() const;
    virtual TransitionType act(const Vector &in, Vector *out) const;
};

}

#endif /* GRL_RANDOM_POLICY_H_ */
