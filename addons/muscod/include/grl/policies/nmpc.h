/** \file nmpc.h
 * \brief NMPC policy header file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-05-08
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

#ifndef GRL_NMPC_POLICY_H_
#define GRL_NMPC_POLICY_H_

#include <grl/policy.h>

class MUSCOD;

namespace grl
{

/// NMPC policy
class NMPCPolicy : public Policy
{
  public:
    TYPEINFO("policy/nmpc")

  protected:
    MUSCOD *muscod_;
    std::string model_path_, model_name_;
    size_t outputs_;
    double tau_;
    
    // NOTE: Non-autonomous policy will have trouble being visualized
    mutable double time_;

  public:
    NMPCPolicy() : muscod_(NULL), outputs_(1), tau_(0.05), time_(0.) { }
    ~NMPCPolicy();
  
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Policy
    virtual NMPCPolicy *clone() const;
    virtual void act(const Vector &in, Vector *out) const;
    virtual void act(const Vector &prev_in, const Vector &prev_out, const Vector &in, Vector *out) const;
};

}

#endif /* GRL_NMPC_POLICY_H_ */
