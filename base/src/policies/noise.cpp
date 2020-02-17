/** \file noise.cpp
 * \brief Post-policy that injects noise source file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2016-01-21
 *
 * \copyright \verbatim
 * Copyright (c) 2016, Wouter Caarls
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

#include <grl/policies/noise.h>

using namespace grl;

REGISTER_CONFIGURABLE(NoisePolicy)

void NoisePolicy::request(ConfigurationRequest *config)
{
  config->push_back(CRP("sigma", "Standard deviation of Gaussian exploration distribution", sigma_, CRP::Configuration));
  config->push_back(CRP("theta", "Ornstein-Uhlenbeck friction term (1=pure Gaussian noise)", theta_, CRP::Configuration));

  config->push_back(CRP("decay_rate", "Multiplicative decay factor per episode", decay_rate_, CRP::Configuration));
  config->push_back(CRP("decay_min", "Minimum decay (sigma_min = sigma*decay_min)", decay_min_, CRP::Configuration));

  config->push_back(CRP("output_min", "vector.action_min", "Lower limit on outputs", min_, CRP::System));
  config->push_back(CRP("output_max", "vector.action_max", "Upper limit on outputs", max_, CRP::System));

  config->push_back(CRP("policy", "mapping/policy", "Deterministic policy to inject noise into", policy_));
}

void NoisePolicy::configure(Configuration &config)
{
  policy_ = (Policy*)config["policy"].ptr();

  min_ = config["output_min"].v();
  max_ = config["output_max"].v();
  if (min_.size() != max_.size() || !min_.size())
    throw bad_param("policy/post/noise:{output_min,output_max}");

  sigma_ = config["sigma"].v();
  if (!sigma_.size())
    sigma_ = ConstantVector(min_.size(), 0.);
    
  if (sigma_.size() == 1 && min_.size() != 1)
    sigma_ = ConstantVector(min_.size(), sigma_[0]);
    
  if (sigma_.size() != min_.size())
    throw bad_param("policy/noise:sigma");
  
  theta_ = config["theta"].v();
  if (!theta_.size())
    theta_ = ConstantVector(min_.size(), 1.);
    
  if (theta_.size() == 1 && min_.size() != 1)
    theta_ = ConstantVector(min_.size(), theta_[0]);
    
  if (theta_.size() != min_.size())
    throw bad_param("policy/noise:theta");

  decay_rate_ = config["decay_rate"];
  decay_min_ = config["decay_min"];
  decay_ = 1.;
}

void NoisePolicy::reconfigure(const Configuration &config)
{
  if (config.has("action") && config["action"].str() == "reset")
    decay_ = 1;
}

void NoisePolicy::act(const Observation &in, Action *out) const
{
  policy_->act(in, out);
  
  Vector mu = *out;
  for (size_t ii=0; ii < out->size(); ++ii)
    (*out)[ii] = fmin(fmax((*out)[ii] + RandGen::getNormal(0., decay_*sigma_[ii]), min_[ii]), max_[ii]);
  out->type = atExploratory;
  out->logp = lognormal(out->v-mu, decay_*sigma_);
}

void NoisePolicy::act(double time, const Observation &in, Action *out)
{
  if (time == 0.)
    decay_ = fmax(decay_*decay_rate_, decay_min_);

  policy_->act(time, in, out);
  
  if (time == 0 || n_.size() != out->size())
    n_ = ConstantVector(out->size(), 0.);
  
  Vector mu = *out;
  for (size_t ii=0; ii < out->size(); ++ii)
  {
    n_[ii] = (1-theta_[ii])*n_[ii] + RandGen::getNormal(0., decay_*sigma_[ii]);
    (*out)[ii] = fmin(fmax((*out)[ii] + n_[ii], min_[ii]), max_[ii]);
  }
  out->type = atExploratory;
  out->logp = lognormal(out->v-mu, decay_*sigma_);
}
