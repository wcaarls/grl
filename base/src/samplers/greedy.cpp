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
//#include <grl/grl.h> // #merge

using namespace grl;

REGISTER_CONFIGURABLE(GreedySampler)
REGISTER_CONFIGURABLE(EpsilonGreedySampler)

void GreedySampler::request(ConfigurationRequest *config)
{
  config->push_back(CRP("rand_max", "In case of multiple maximumum values select a random index among them", rand_max_, CRP::System, 0, 1));
}

void GreedySampler::configure(Configuration &config)
{
  rand_ = new Rand();
  rand_max_ = config["rand_max"];
}

void GreedySampler::reconfigure(const Configuration &config)
{
}

GreedySampler *GreedySampler::clone()
{
  GreedySampler *gs = new GreedySampler(*this);
  gs->rand_ = rand_->clone();

  return gs;
}

size_t GreedySampler::sample(const LargeVector &values, TransitionType &tt)
{
  size_t mai = 0;

  for (size_t ii=1; ii < values.size(); ++ii)
    if (values[ii] > values[mai])
      mai = ii;

  if (rand_max_)
  {
    Vector same_values = ConstantVector(values.size(), 0);
    size_t jj = 0;
    for (size_t ii=0; ii < values.size(); ++ii)
      if (values[ii] == values[mai])
        same_values[jj++] = ii;

    if (jj != 0)
      mai = same_values[rand_->getInteger(jj)];
  }

  tt = ttGreedy;
  return mai;
}

void GreedySampler::distribution(const LargeVector &values, LargeVector *distribution)
{
  TransitionType tt;
  *distribution = LargeVector::Constant(values.size(), 0.);
  (*distribution)[GreedySampler::sample(values, tt)] = 1;
}

void EpsilonGreedySampler::request(ConfigurationRequest *config)
{
  GreedySampler::request(config);
  config->push_back(CRP("epsilon", "Exploration rate", epsilon_, CRP::Online));
}

void EpsilonGreedySampler::configure(Configuration &config)
{
  GreedySampler::configure(config);
  
  epsilon_ = config["epsilon"];
}

void EpsilonGreedySampler::reconfigure(const Configuration &config)
{
  config.get("epsilon", epsilon_);
}

EpsilonGreedySampler *EpsilonGreedySampler::clone()
{
  EpsilonGreedySampler *egs = new EpsilonGreedySampler(*this);
  egs->rand_ = rand_->clone();
  
  return egs;
}

size_t EpsilonGreedySampler::sample(const LargeVector &values, TransitionType &tt)
{
  if (rand_->get() < epsilon_)
  {
    tt = ttExploratory;
    return rand_->getInteger(values.size());
  }

  return GreedySampler::sample(values, tt);
}

void EpsilonGreedySampler::distribution(const LargeVector &values, LargeVector *distribution)
{
  GreedySampler::distribution(values, distribution);

  for (size_t ii=0; ii < values.size(); ++ii)
  {
    if ((*distribution)[ii] == 1)
      (*distribution)[ii] = 1-epsilon_;
    (*distribution)[ii] += epsilon_/values.size();
  }
}
