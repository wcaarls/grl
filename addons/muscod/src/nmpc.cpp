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
  // muscod_->sqpInitialize(muscod_->getSSpec(), NULL, NULL);
  muscod_->setLogLevelScreen(-1);
  muscod_->setLogLevelAndFile(-1, NULL, NULL);
  // muscod_->sqpSolve();
  // std::cout << "MUSCOD is ready!" << std::endl;

  // get proper dimensions
  muscod_->nmpcInitialize(muscod_->getSSpec(), NULL, NULL);
  // solve until convergence to prepare solver
  for (int ii=0; ii < 20; ++ii)
  {
    muscod_->nmpcFeedback(NULL, NULL, NULL);
    muscod_->nmpcTransition();
    muscod_->nmpcPrepare();
  }

  // save solver state
  data_.backup_muscod_state(muscod_);

  // signal readiness
  // std::cout << "MUSCOD is ready!" << std::endl;
}

void NMPCPolicy::reconfigure(const Configuration &config)
{
}


void NMPCPolicy::muscod_reset()
{
  // load solution state
  data_.restore_muscod_state(muscod_);

  // solve until convergence to prepare solver
  // muscod_->nmpcInitialize(muscod_->getSSpec(), NULL, NULL);
  // muscod_->setLogLevelScreen(-1);
  // muscod_->setLogLevelAndFile(-1, NULL, NULL);
  // muscod_->nmpcPrepare();
  // muscod_->options->wflag = MS_WARM;
  // muscod_->sqpSolve();

  // solve until convergence to prepare solver
  for (int ii=0; ii < 20; ++ii)
  {
    muscod_->nmpcFeedback(NULL, NULL, NULL);
    muscod_->nmpcTransition();
    muscod_->nmpcPrepare();
  }

  // signal readiness
  // std::cout << "MUSCOD is reseted!" << std::endl;
}

NMPCPolicy *NMPCPolicy::clone() const
{
  return NULL;
}

void NMPCPolicy::act(double time, const Vector &in, Vector *out)
{
  if (time == 0.0)
  {
    // muscod_->nmpcInitialize(muscod_->getSSpec(), NULL, NULL);
    // muscod_->setLogLevelScreen(-1);
    // muscod_->setLogLevelAndFile(-1, NULL, NULL);
    // muscod_->nmpcPrepare();
    // muscod_->options->wflag = MS_WARM;
    muscod_reset();
  }

  out->resize(outputs_);
  for (int ii=0; ii < 3; ++ii)
  {
    muscod_->nmpcFeedback(in.data(), NULL, out->data());
    muscod_->nmpcTransition();
    muscod_->nmpcShift(3);
    muscod_->nmpcPrepare();
  }
}
