/** \file nmpc_stages.h
 * \brief NMPC Stages policy header file.
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

#ifndef GRL_NMPC_STAGES_POLICY_H_
#define GRL_NMPC_STAGES_POLICY_H_

#include <grl/policy.h>
#include <grl/policies/muscod_nmpc.h>
#include <grl/policies/nmpc.h>

class MUSCOD;

namespace grl
{

/// NMPC Stages policy
class NMPCStagesPolicy : public NMPCPolicy
{
  public:
    TYPEINFO("policy/nmpc_stages", "Nonlinear model predictive control policy using the MUSCOD library")

  public:
    //NMPCPolicy() :  { }
    //~NMPCPolicy();

    // From Policy
    virtual TransitionType act(double time, const Vector &in, Vector *out);
};

}

#endif /* GRL_NMPC_STAGES_POLICY_H_ */
