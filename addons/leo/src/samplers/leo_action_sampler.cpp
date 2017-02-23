/** \file leo_action_sampler.cpp
 * \brief Wrapper for an action sampler for Leo which supports contact signals (source file).
 *
 * \author    Ivan Koryakovskiy <i.koryakovskiy@tudelft.nl>
 * \date      2016-09-30
 *
 * \copyright \verbatim
 * Copyright (c) 2016, Ivan Koryakovskiy
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
#include <grl/samplers/leo_action_sampler.h>
#include <leo.h>

using namespace grl;

REGISTER_CONFIGURABLE(LeoActionSampler)

void LeoActionSampler::request(ConfigurationRequest *config)
{
  Sampler::request(config);

  config->push_back(CRP("sampler", "sampler", "Samples actions from action-values", sampler_));
  config->push_back(CRP("sub_ic_signal", "signal/vector", "Subscrider to the initialization and contact signal", sub_ic_signal_, true));
  config->push_back(CRP("pub_sub_sampler_state", "signal/vector", "Publisher and subscriber of the sampler state with memory such as previous action, noise, etc.", pub_sub_sampler_state_, true));
}

void LeoActionSampler::configure(Configuration &config)
{
  Sampler::configure(config);

  sampler_ = (Sampler*)config["sampler"].ptr();
  pub_sub_sampler_state_ = (VectorSignal*)config["pub_sub_sampler_state"].ptr();
  sub_ic_signal_ = (VectorSignal*)config["sub_ic_signal"].ptr();
}

void LeoActionSampler::reconfigure(const Configuration &config)
{
  Sampler::reconfigure(config);
}

size_t LeoActionSampler::sample(double time, const LargeVector &values, ActionType *at)
{
  if (sub_ic_signal_)
  {
    Vector signal = sub_ic_signal_->get();
    Vector sampler_state = pub_sub_sampler_state_->get(); // must hold either actual values of action or noise (not indexed arrays)
    if ((int)signal[0] & lstSwlTouchDown)
    {
      // Take care of Leo body symmetry, if required!
      TRACE(sampler_state);
      Vector sampler_state_new = sampler_state;
      Vector ti_actuator_to = signal.block(0, 1, 1, CLeoBhBase::svNumActions);
      TRACE(ti_actuator_to);
      Vector ti_actuator_from = signal.block(0, 1+CLeoBhBase::svNumActions, 1, CLeoBhBase::svNumActions);
      TRACE(ti_actuator_from);

      // Weak point of the implementation: auto-actuated knee is not supported properly.
      // A simplification is made: if knee is autoactuated, then at the moment of contact
      // knee action or noise is set to 0 (iven if noise is centered not around 0)
      // Good thing to know is that auto-actuated knee action is usually small, around 0,
      // therefore if PADA selects a new action, 0 is a good choice.
      for (int i = 0; i < CLeoBhBase::svNumActions; i++)
      {
        if (ti_actuator_from[i] != -1)
        {
          if (i > 0 && ti_actuator_from[i] == ti_actuator_from[i-1])
            sampler_state_new[ti_actuator_to[i]] = 0; // two same consecutive indexies mean that knee is autoactuated
          else
            sampler_state_new[ti_actuator_to[i]] = sampler_state[ti_actuator_from[i]];
        }
      }
      TRACE(sampler_state_new);

      // update memory which will be used in the sample() call below
      pub_sub_sampler_state_->set(sampler_state_new);
    }
  }

  return sampler_->sample(time, values, at);
}

void LeoActionSampler::distribution(const LargeVector &values, LargeVector *distribution) const
{
  sampler_->distribution(values, distribution);
}
