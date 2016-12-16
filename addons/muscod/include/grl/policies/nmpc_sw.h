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

#ifndef GRL_NMPC_SW_POLICY_H_
#define GRL_NMPC_SW_POLICY_H_

#include <grl/policy.h>
#include <grl/policies/muscod_nmpc.h>

class MUSCOD;

namespace grl
{

/// NMPC policy
class NMPC_SWPolicy : public Policy
{
  public:
    TYPEINFO("mapping/policy/nmpc_sw", "Nonlinear model predictive control policy for the simplest walker using the MUSCOD library")

  protected:
    int verbose_;

    // MUSCOD-II interface
    MUSCOD *muscod_nmpc_;
    NMPCProblem *nmpc_;
    std::string nmpc_model_name_, model_name_;
    size_t outputs_;
    Vector initial_sd_, initial_pf_, initial_qc_, final_sd_;

  public:
    NMPC_SWPolicy() : muscod_nmpc_(NULL), outputs_(1), verbose_(false) { }
    ~NMPC_SWPolicy();

    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);
    virtual void muscod_reset(const Vector &initial_obs, double time);

    // From Policy
    virtual void act(double time, const Vector &in, Vector *out);

    // Own
    void *setup_model_path(const std::string path, const std::string model, const std::string lua_model);
};

}

#endif /* GRL_NMPC_SW_POLICY_H_ */
