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
//#include <grl/policies/muscod_data.h> // MUSCOD-II thread-safe data structure
#include <grl/policies/muscod_mhe.h>
#include <grl/policies/muscod_nmpc.h>

class MUSCOD;

namespace grl
{

typedef void (* t_obs_converter)(const double *from, double *to);

/// NMPC policy with moving horizon estimator (MHE)
class MHE_NMPCPolicy : public Policy
{
  public:
    TYPEINFO("policy/mhe_nmpc", "Nonlinear model predictive control policy with moving horizon estimator using the MUSCOD library")

  protected:
    int verbose_;

    // MUSCOD-II interface
    t_obs_converter so_convert_obs_for_muscod_;//(*so_convert_obs_for_muscod)(const double *from, double *to);
//    MuscodData data_;
    MUSCOD *muscod_mhe_, *muscod_nmpc_;
    MHEProblem *mhe_;
    NMPCProblem *nmpc_;
    std::string mhe_model_name_, nmpc_model_name_, lua_model_;
    size_t outputs_;
    Vector real_pf_, initial_sd_, initial_pf_, initial_qc_, final_sd_, hs_, ss_;

  public:
    MHE_NMPCPolicy() : muscod_mhe_(NULL), muscod_nmpc_(NULL), outputs_(1), verbose_(false) { }
    ~MHE_NMPCPolicy();

    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);
    virtual void muscod_reset(Vector &initial_obs, double time);

    // From Policy
    virtual MHE_NMPCPolicy *clone() const;
    virtual void act(double time, const Vector &in, Vector *out);

    // Own
    void *setup_model_path(const std::string path, const std::string model, const std::string lua_model);
};

}

#endif // /* GRL_MHE_NMPC_H_ */
