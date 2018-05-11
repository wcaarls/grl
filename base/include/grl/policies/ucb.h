/** \file ucb.h
 * \brief UCB policy header file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-08-20
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

#ifndef GRL_UCB_POLICY_H_
#define GRL_UCB_POLICY_H_

#include <grl/policy.h>
#include <grl/discretizer.h>
#include <grl/projector.h>
#include <grl/representation.h>
#include <grl/sampler.h>

namespace grl
{

/// Policy based on the upper confidence bound of an action-value Representation.
class UCBPolicy : public ValuePolicy
{
  public:
    TYPEINFO("mapping/policy/discrete/value/ucb", "UCB1 policy")

  protected:
    Discretizer *discretizer_;
    Projector *projector_;
    Representation *representation_, *visit_representation_;
    double c_p_;

  public:
    UCBPolicy() : discretizer_(NULL), projector_(NULL), representation_(NULL), visit_representation_(NULL), c_p_(sqrt(2)) { }
  
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Policy
    virtual void act(const Observation &in, Action *out) const;
    virtual void act(double time, const Observation &in, Action *out);
    
    // From DiscretePolicy
    virtual void distribution(const Observation &in, const Action &prev, LargeVector *out) const;
    
    // From ValuePolicy
    virtual double value(const Observation &in) const;

  protected:
    virtual void values(const Observation &in, LargeVector *out) const;
};

}

#endif /* GRL_UCB_POLICY_H_ */
