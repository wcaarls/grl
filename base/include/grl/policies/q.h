/** \file q.h
 * \brief Q policy header file.
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

#ifndef GRL_Q_POLICY_H_
#define GRL_Q_POLICY_H_

#include <grl/policy.h>
#include <grl/discretizer.h>
#include <grl/projector.h>
#include <grl/representation.h>
#include <grl/sampler.h>

namespace grl
{

/// Policy based on an action-value Representation.
class QPolicy : public ValuePolicy
{
  public:
    TYPEINFO("mapping/policy/discrete/value/q", "Q-value based policy")

  protected:
    Discretizer *discretizer_;
    Projector *projector_;
    Representation *representation_;
    Sampler *sampler_;

  public:
    QPolicy() : discretizer_(NULL), projector_(NULL), representation_(NULL), sampler_(NULL) { }
  
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

/// Policy based on a vectorized action-value Representation.
class QVectorPolicy : public QPolicy
{
  public:
    TYPEINFO("mapping/policy/discrete/value/qvector", "Vectorized Q-value based policy")
    
  public:
    // From Configurable
    virtual void request(ConfigurationRequest *config);

  protected:
    // From QPolicy
    virtual void values(const Observation &in, LargeVector *out) const;
};

}

#endif /* GRL_Q_POLICY_H_ */
