/** \file lowpass.cpp
 * \brief Post-policy that low-pass filters the action source file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2020-07-23
 *
 * \copyright \verbatim
 * Copyright (c) 2020, Wouter Caarls
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

#include <grl/policies/lowpass.h>

using namespace grl;

REGISTER_CONFIGURABLE(LowPassPolicy)

void LowPassPolicy::request(ConfigurationRequest *config)
{
  config->push_back(CRP("tau", "Update factor for exponential moving average filter", tau_, CRP::Configuration));

  config->push_back(CRP("policy", "mapping/policy", "Deterministic policy to low-pass filter", policy_));
}

void LowPassPolicy::configure(Configuration &config)
{
  policy_ = (Policy*)config["policy"].ptr();
  tau_ = config["tau"];
}

void LowPassPolicy::reconfigure(const Configuration &config)
{
}

void LowPassPolicy::act(const Observation &in, Action *out) const
{
  policy_->act(in, out);
}

void LowPassPolicy::act(double time, const Observation &in, Action *out)
{
  policy_->act(time, in, out);

  if (time == 0.)
    value_ = *out;
  
  value_ = tau_*out->v + (1-tau_)*value_;
  out->v = value_;
}
