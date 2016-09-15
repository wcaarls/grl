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
#include <grl/policies/muscod_nmpc.h>

class MUSCOD;

namespace grl
{

/// NMPC policy
class NMPCPolicy : public Policy
{
  public:
    TYPEINFO("policy/nmpc", "Nonlinear model predictive control policy using the MUSCOD library")

  protected:
    int verbose_;
    int initFeedback_;

    // MUSCOD-II interface
    MUSCOD *muscod_;
    NMPCProblem *nmpc_;
    const std::string thread_id_ = "";
    pthread_t thread_;
    pthread_cond_t cond_iv_ready_;
    pthread_mutex_t mutex_;

    std::string model_name_, lua_model_, nmpc_model_name_;
    size_t outputs_;
    Vector initial_sd_, initial_pf_, initial_qc_, final_sd_;

    // CONTROL LOOP
    bool iv_provided_;
    bool qc_retrieved_;

    // relative path to model directory
    // NOTE will end up next to the DAT file as 'run_nmpc.bin'!
    const std::string restart_path_ = "";
    const std::string restart_name_ = "run_nmpc";

    // GRL
    Vector action_min_, action_max_;

  public:
    NMPCPolicy() : muscod_(0), nmpc_(0), outputs_(1), verbose_(0), initFeedback_(0) { };
    ~NMPCPolicy();

    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);
    virtual void muscod_reset(const Vector &initial_obs, const Vector &initial_pf, Vector &initial_qc) const;

    // From Policy
    virtual NMPCPolicy *clone() const;
    virtual TransitionType act(double time, const Vector &in, Vector *out);

    // Own
    void *setup_model_path(const std::string path, const std::string model, const std::string lua_model);
};

}

#endif /* GRL_NMPC_POLICY_H_ */
