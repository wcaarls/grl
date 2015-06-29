/** \file greedy.cpp
 * \brief Greedy and Epsilon-greedy samplers source file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-01-22
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
#include <grl/samplers/greedy.h>

using namespace grl;

REGISTER_CONFIGURABLE(GreedySampler)
REGISTER_CONFIGURABLE(EpsilonGreedySampler)

void GreedySampler::request(ConfigurationRequest *config)
{
}

void GreedySampler::configure(Configuration &config)
{
  rand_ = new Rand();
}

void GreedySampler::reconfigure(const Configuration &config)
{
}

GreedySampler *GreedySampler::clone()
{
  GreedySampler *gs = new GreedySampler();
  gs->rand_ = rand_->clone();

  return gs;
}

size_t GreedySampler::sample(const Vector &values) const
{
  size_t mai = 0;

  for (size_t ii=1; ii < values.size(); ++ii)
    if (values[ii] > values[mai])
      mai = ii;
/*
 * Commented code selects random index in case of multiple maximumum values
 *
  Vector same_values(values.size(), 0);
  size_t jj = 0;
  for (size_t ii=0; ii < values.size(); ++ii)
    if (values[ii] == values[mai])
      same_values[jj++] = ii;

  if (jj != 0)
    mai = same_values[rand_->getInteger(jj)];
*/
  return mai;
}

void GreedySampler::distribution(const Vector &values, Vector *distribution) const
{
  size_t mai = GreedySampler::sample(values);
  distribution->resize(values.size());

  for (size_t ii=0; ii < values.size(); ++ii)
  {
    if (ii == mai)
      (*distribution)[ii] = 1;
    else
      (*distribution)[ii] = 0;
  }
}

void EpsilonGreedySampler::request(ConfigurationRequest *config)
{
  config->push_back(CRP("epsilon", "Exploration rate", epsilon_));
}

void EpsilonGreedySampler::configure(Configuration &config)
{
  epsilon_ = config["epsilon"];
  rand_ = new Rand();
}

void EpsilonGreedySampler::reconfigure(const Configuration &config)
{
}

EpsilonGreedySampler *EpsilonGreedySampler::clone()
{
  EpsilonGreedySampler *egs = new EpsilonGreedySampler();
  egs->epsilon_ = epsilon_;
  egs->rand_ = rand_->clone();
  
  return egs;
}

size_t EpsilonGreedySampler::sample(const Vector &values) const
{
  if (rand_->get() < epsilon_)
    return rand_->getInteger(values.size());

  return GreedySampler::sample(values);
}

void EpsilonGreedySampler::distribution(const Vector &values, Vector *distribution) const
{
  GreedySampler::distribution(values, distribution);

  for (size_t ii=0; ii < values.size(); ++ii)
  {
    if ((*distribution)[ii] == 1)
      (*distribution)[ii] = 1-epsilon_;
    (*distribution)[ii] += epsilon_/values.size();
  }
}
