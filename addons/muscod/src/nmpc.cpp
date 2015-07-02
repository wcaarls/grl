/** \file nmpc.cpp
 * \brief NMPC policy source file.
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

#include <grl/policies/nmpc.h>
#include <wrapper.hpp>

using namespace grl;

REGISTER_CONFIGURABLE(NMPCPolicy)

NMPCPolicy::~NMPCPolicy()
{
  safe_delete(&muscod_);
}

void NMPCPolicy::request(ConfigurationRequest *config)
{
  config->push_back(CRP("model_path", "Path to MUSCOD model library", model_path_));
  config->push_back(CRP("model_name", "Name of MUSCOD model library", model_name_));
  config->push_back(CRP("outputs", "int.action_dims", "Number of outputs", (int)outputs_, CRP::System, 1));
}

void NMPCPolicy::configure(Configuration &config)
{
  muscod_ = new MUSCOD();

  model_path_ = config["model_path"].str();
  model_name_ = config["model_name"].str();
  outputs_ = config["outputs"];
        
  muscod_->setModelPathAndName(model_path_.c_str(), model_name_.c_str());
  muscod_->loadFromDatFile(NULL, NULL);
}

void NMPCPolicy::reconfigure(const Configuration &config)
{
}

NMPCPolicy *NMPCPolicy::clone() const
{
  return NULL;
}

void NMPCPolicy::act(double time, const Vector &in, Vector *out)
{
  if (time == 0.)
    muscod_->nmpcInitialize(muscod_->getSSpec(), NULL, NULL);

  out->resize(outputs_);
  for (int ii=0; ii < 3; ++ii)
  {
    muscod_->nmpcFeedback(in.data(), out->data());    
    muscod_->nmpcTransition();
    muscod_->nmpcPrepare();
  }
  
  DVec pf_lo = muscod_->data.dataMSSQP->vbnd.lo.pf;
  DVec pf_up = muscod_->data.dataMSSQP->vbnd.up.pf;
  for (int ii = 0; ii < muscod_->data.dataMSSQP->vdim.pf; ++ii) {
      V_EACC(pf_up, ii) = time;
      V_EACC(pf_lo, ii) = time;
  }
  scale_pf(pf_lo);
  scale_pf(pf_up);
}
