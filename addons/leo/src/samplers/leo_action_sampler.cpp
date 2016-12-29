/** \file leo_action_sampler.cpp
 * \brief Wrapper for an action sampler for Leo which supports contact signals (source file).
 *
 * \author    Ivan Koryakovskiy <i.koryakovskiy@tudelft.nl>
 * \date      2016-09-30
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
#include <grl/grl.h>
#include <grl/environments/leo/samplers/leo_action_sampler.h>
#include <leo.h>

using namespace grl;

REGISTER_CONFIGURABLE(LeoActionSampler)

void LeoActionSampler::request(ConfigurationRequest *config)
{
  Sampler::request(config);

  config->push_back(CRP("sampler", "sampler", "Samples actions from action-values", sampler_));
  config->push_back(CRP("sub_ic_signal", "signal/vector", "Subscrider to the initialization and contact signal", sub_ic_signal_, true));
  config->push_back(CRP("pub_sub_sampler_state", "signal/vector", "Publisher and subscriber of the sampler state with memory such as previous action, noise, etc.", pub_sub_sampler_state_, true));
//  config->push_back(CRP("discretizer", "discretizer.action", "Action discretizer", discretizer_));
}

void LeoActionSampler::configure(Configuration &config)
{
  Sampler::configure(config);

  sampler_ = (Sampler*)config["sampler"].ptr();
  pub_sub_sampler_state_ = (VectorSignal*)config["pub_sub_sampler_state"].ptr();
  sub_ic_signal_ = (VectorSignal*)config["sub_ic_signal"].ptr();
//  discretizer_ = (Discretizer*)config["discretizer"].ptr();
}

void LeoActionSampler::reconfigure(const Configuration &config)
{
  Sampler::reconfigure(config);
}

LeoActionSampler *LeoActionSampler::clone()
{
  LeoActionSampler *egs = new LeoActionSampler(*this);
  return egs;
}

size_t LeoActionSampler::sample(const LargeVector &values, TransitionType &tt)
{
  if (sub_ic_signal_)
  {
    LargeVector signal = sub_ic_signal_->get();
    LargeVector memory = pub_sub_sampler_state_->get(); // must hold either actual values of action or noise (not indexed arrays)
    if (signal[lisdTypeInit] == lstContact)
    {
      // Takes care of Leo body symmetry
      TRACE (memory);
      double tmp = memory[0];
      memory[0] = memory[1];              // hipright
      memory[1] = tmp;                    // hipleft
      memory[2] = signal[lisdSwingKnee];  // kneeleft
      TRACE (memory);

      // update memory which will be used in the sample() call below
      pub_sub_sampler_state_->set(memory);
    }
  }

  return sampler_->sample(values, tt);
}

void LeoActionSampler::distribution(const LargeVector &values, LargeVector *distribution)
{
  sampler_->distribution(values, distribution);
}


