/** \file v.h
 * \brief V policy header file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-08-28
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

#ifndef GRL_V_POLICY_H_
#define GRL_V_POLICY_H_

#include <grl/policy.h>
#include <grl/discretizer.h>
#include <grl/environments/observation.h>
#include <grl/projector.h>
#include <grl/representation.h>
#include <grl/sampler.h>

namespace grl
{

/// Policy based on a state-value Representation.
class VPolicy : public ValuePolicy
{
  public:
    TYPEINFO("mapping/policy/value/v", "State-value based policy")

  protected:
    Discretizer *discretizer_;
    ObservationModel *model_;
    Projector *projector_;
    Representation *representation_;
    Sampler *sampler_;
    
    double gamma_;

  public:
    VPolicy() : discretizer_(NULL), model_(NULL), projector_(NULL), representation_(NULL), sampler_(NULL), gamma_(0.97) { }
  
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Policy
    virtual void act(const Observation &in, Action *out) const;
    virtual void distribution(const Observation &in, const Action &prev, LargeVector *out) const;
    
    // From ValuePolicy
    virtual double value(const Observation &in) const;
    
  protected:
    virtual void values(const Observation &in, LargeVector *out) const;
};

}

#endif /* GRL_V_POLICY_H_ */
