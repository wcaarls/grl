/** \file noise.h
 * \brief Post-policy that low-pass filters the action header file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2020-07-23
 *
 * \copyright \verbatim
 * Copyright (c) 2020, Wouter Caarls
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

#ifndef GRL_LOWPASS_POLICY_H_
#define GRL_LOWPASS_POLICY_H_

#include <grl/policy.h>

namespace grl
{

/// Post-policy that low-pass filters the action.
class LowPassPolicy : public Policy
{
  public:
    TYPEINFO("mapping/policy/post/lowpass", "Postprocesses policy output by low-pass filtering")

  protected:
    Policy *policy_;
    double tau_;
    
    Vector value_;

  public:
    LowPassPolicy() : policy_(NULL), tau_(1.) { }
    
    // From Configurable  
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Policy
    virtual void act(const Observation &in, Action *out) const;
    virtual void act(double time, const Observation &in, Action *out);
};

}

#endif /* GRL_LOWPASS_POLICY_H_ */
