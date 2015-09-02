/** \file softmax.cpp
 * \brief Greedy and Epsilon-softmax samplers source file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-07-01
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
#include <grl/samplers/softmax.h>

using namespace grl;

REGISTER_CONFIGURABLE(SoftmaxSampler)

void SoftmaxSampler::request(ConfigurationRequest *config)
{
  config->push_back(CRP("tau", "Temperature of Boltzmann distribution", tau_, CRP::Online, 0.001, 100.));
}

void SoftmaxSampler::configure(Configuration &config)
{
  tau_ = config["tau"];
}

void SoftmaxSampler::reconfigure(const Configuration &config)
{
  config.get("tau", tau_);
}

SoftmaxSampler *SoftmaxSampler::clone()
{
  return new SoftmaxSampler(*this);
}

size_t SoftmaxSampler::sample(const Vector &values) const
{
  Vector dist;
  
  distribution(values, &dist);
  
  return ::sample(dist, 1.);
}

void SoftmaxSampler::distribution(const Vector &values, Vector *distribution) const
{
  Vector v(values.size());
  double sum=0;

  for (size_t ii=0; ii < values.size(); ++ii)
  {
    v[ii] = exp(values[ii]/tau_);
    sum += v[ii];
  }

  distribution->resize(v.size());
  for (size_t ii=0; ii < values.size(); ++ii)
    (*distribution)[ii] = v[ii]/sum;
}
