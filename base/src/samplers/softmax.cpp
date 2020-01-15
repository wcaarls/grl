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
  config->push_back(CRP("tau", "Temperature of Boltzmann distribution", tau_, CRP::Online, 0.000001, 100.));
}

void SoftmaxSampler::configure(Configuration &config)
{
  tau_ = config["tau"];
}

void SoftmaxSampler::reconfigure(const Configuration &config)
{
  config.get("tau", tau_);
}

size_t SoftmaxSampler::sample(const LargeVector &values, ActionType *at, double *logp) const
{
  LargeVector dist;
  
  distribution(values, &dist);
  
  if (at)
    *at = atExploratory;
    
  size_t a = grl::sample(dist, 1.);
  
  if (logp)
    *logp = std::log(dist[a]);
  
  return a;
}

void SoftmaxSampler::distribution(const LargeVector &values, LargeVector *distribution) const
{
  LargeVector v = LargeVector::Zero(values.size());
  for (size_t ii=0; ii < values.size(); ++ii)
    if (std::isnan(values[ii]))
      ERROR("SoftmaxSampler: NaN value in Boltzmann distribution 1");

  distribution->resize(values.size());
  const double threshold = -100;

  // Find max_power and min_power, and center of feasible powers
  double max_power = -DBL_MAX;
  for (size_t ii=0; ii < values.size(); ++ii)
  {
    double p = values[ii]/tau_;
    max_power = (max_power < p) ? p : max_power;
  }
  double min_power = max_power + threshold;
  double center = (max_power+min_power)/2.0;

  // Discard powers from interval [0.0; threshold] * max_power
  double sum = 0;
  for (size_t ii=0; ii < values.size(); ++ii)
  {
    double p = values[ii]/tau_;
    if (p > min_power)
    {
      p -= center;
      v[ii] = exp(p);
      sum += v[ii];
      (*distribution)[ii] = 1;

      if (std::isnan(v[ii]))
        ERROR("SoftmaxSampler: NaN value in Boltzmann distribution 2");
    }
    else
      (*distribution)[ii] = 0;
  }

  for (size_t ii=0; ii < values.size(); ++ii)
  {
    (*distribution)[ii] *= v[ii]/sum;
    if (std::isnan((*distribution)[ii]))
      ERROR("SoftmaxSampler: NaN value in Boltzmann distribution 4");
  }
}
