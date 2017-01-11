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

#ifndef GRL_MHE_NMPC_H_
#define GRL_MHE_NMPC_H_

#include <grl/policy.h>
#include <grl/policies/nmpc_base.h>
#include <grl/policies/muscod_mhe.h>
#include <grl/policies/muscod_nmpc.h>


class MUSCOD;

namespace grl
{

/// NMPC policy with moving horizon estimator (MHE)
class MHE_NMPCPolicy : public NMPCBase
{
  public:
    TYPEINFO("mapping/policy/mhe_nmpc", "Nonlinear model predictive control policy with moving horizon estimator using the MUSCOD library")

  protected:
    MUSCOD *muscod_mhe_, *muscod_nmpc_;
    MHEProblem *mhe_;
    NMPCProblem *nmpc_;
    const std::string thread_id_ = "";
    pthread_t thread_;
    pthread_cond_t cond_iv_ready_;
    pthread_mutex_t mutex_;

    Vector initial_sd_, initial_pf_, initial_qc_, final_sd_, hs_, ss_;

    // CONTROL LOOP
    bool iv_provided_;
    bool qc_retrieved_;

    // relative path to model directory
    // NOTE will end up next to the DAT file as 'run_nmpc.bin'!
    const std::string restart_path_ = "";
    const std::string restart_name_ = "run_nmpc";

    std::string feedback_;
    int n_iter_;

  public:
    MHE_NMPCPolicy() : muscod_mhe_(NULL), muscod_nmpc_(NULL), mhe_(NULL), nmpc_(NULL), n_iter_(1) { }
    ~MHE_NMPCPolicy();

    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);
    virtual void muscod_reset(const Vector &initial_obs, const Vector &initial_pf, Vector &initial_qc);

    // From Policy
    virtual TransitionType act(double time, const Vector &in, Vector *out);
};

}

#endif // /* GRL_MHE_NMPC_H_ */
