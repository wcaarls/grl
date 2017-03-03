/** \file feed_forward.h
 * \brief Feed forward policy header file.
 *
 * \author    Ivan Koryakovskiy <i.koryakovskiy@gmail.com>
 * \date      2016-02-11
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

#ifndef GRL_FEED_FORWARD_POLICY_H_
#define GRL_FEED_FORWARD_POLICY_H_

#include <grl/policy.h>

namespace grl
{

/// FF policy
class FeedForwardPolicy : public Policy
{
  public:
    TYPEINFO("mapping/policy/feed_forward", "Feed-forward policy")

  protected:
    Mapping *controls_;

  public:
    FeedForwardPolicy() : controls_(NULL) { }
  
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Policy
    virtual void act(double time, const Observation &in, Action *out);
};

}

#endif /* GRL_FEED_FORWARD_POLICY_H_ */
