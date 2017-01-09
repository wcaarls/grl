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

  config->push_back(CRP("policy", "mapping/policy", "Policy to inject noise into", policy_));
}

void NoisePolicy::configure(Configuration &config)
{
  policy_ = (Policy*)config["policy"].ptr();
  
  sigma_ = config["sigma"].v();
  theta_ = config["theta"].v();
}

void NoisePolicy::reconfigure(const Configuration &config)
{
}

TransitionType NoisePolicy::act(const Vector &in, Vector *out) const
{
  policy_->act(in, out);
  
  if (!sigma_.size())
    sigma_ = ConstantVector(out->size(), 0.);
    
  if (sigma_.size() == 1 && out->size() != 1)
    sigma_ = ConstantVector(out->size(), sigma_[0]);
    
  if (sigma_.size() != out->size())
    throw bad_param("policy/noise:sigma");
  
  for (size_t ii=0; ii < out->size(); ++ii)
    (*out)[ii] += RandGen::getNormal(0., sigma_[ii]);
}

TransitionType NoisePolicy::act(double time, const Vector &in, Vector *out)
{
  policy_->act(in, out);
  
  if (!sigma_.size())
    sigma_ = ConstantVector(out->size(), 0.);
    
  if (sigma_.size() == 1 && out->size() != 1)
    sigma_ = ConstantVector(out->size(), sigma_[0]);
    
  if (sigma_.size() != out->size())
    throw bad_param("policy/noise:sigma");
    
  if (!theta_.size())
    theta_ = ConstantVector(out->size(), 1.);
    
  if (theta_.size() == 1 && out->size() != 1)
    theta_ = ConstantVector(out->size(), theta_[0]);
    
  if (theta_.size() != out->size())
    throw bad_param("policy/noise:theta");
    
  if (time == 0 || n_.size() != out->size())
    n_ = ConstantVector(out->size(), 0.);
  
  for (size_t ii=0; ii < out->size(); ++ii)
  {
    n_[ii] = (1-theta_[ii])*n_[ii] + RandGen::getNormal(0., sigma_[ii]);
    (*out)[ii] += n_[ii];
  }
  return ttExploratory;
}
