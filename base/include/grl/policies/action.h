/** \file action.h
 * \brief Action and action-probability policies header file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-02-16
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

#ifndef GRL_ACTION_POLICY_H_
#define GRL_ACTION_POLICY_H_

#include <grl/policy.h>
#include <grl/projector.h>
#include <grl/representation.h>
#include <grl/discretizer.h>
#include <grl/sampler.h>

namespace grl
{

/// Policy based on a direct action representation
class ActionPolicy : public Policy
{
  public:
    TYPEINFO("mapping/policy/action", "Policy based on a direct action representation")

  protected:
    Projector *projector_;
    Representation *representation_;
    
    Vector min_, max_, sigma_;
    double decay_rate_, decay_min_, decay_;

  public:
    ActionPolicy() : projector_(NULL), representation_(NULL), decay_rate_(1), decay_min_(0), decay_(1) { }
    
    // From Configurable  
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Policy
    virtual void act(double time, const Observation &in, Action *out);
    virtual void act(const Observation &in, Action *out) const;
};

/// Policy based on an action-probability representation.
class ActionProbabilityPolicy : public DiscretePolicy
{
  public:
    TYPEINFO("mapping/policy/discrete/action_probability", "Policy based on an action-probability representation")

  protected:
    Discretizer *discretizer_;
    Projector *projector_;
    Representation *representation_;
    Sampler *sampler_;
    
    std::vector<Vector> variants_;

  public:
    ActionProbabilityPolicy() : discretizer_(NULL), projector_(NULL), representation_(NULL), sampler_(NULL) { }
    ~ActionProbabilityPolicy()
    {
      if (sampler_) delete sampler_;
    }
  
    // From Configurable  
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);
  
    // From Policy
    virtual void act(const Observation &in, Action *out) const;
    
    // From DiscretePolicy
    virtual void distribution(const Observation &in, const Action &prev, LargeVector *out) const;
    
  protected:
    virtual void values(const Observation &in, LargeVector *out) const;
};

}

#endif /* GRL_ACTION_POLICY_H_ */
