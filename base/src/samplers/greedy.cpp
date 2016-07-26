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
#include <grl/grl.h>

using namespace grl;

REGISTER_CONFIGURABLE(GreedySampler)
REGISTER_CONFIGURABLE(EpsilonGreedySampler)
REGISTER_CONFIGURABLE(EpsilonGreedyOUSampler)

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

size_t GreedySampler::sample(const Vector &values, TransitionType &tt) const
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

void GreedySampler::distribution(const Vector &values, Vector *distribution) const
{
  TransitionType tt;
  *distribution = ConstantVector(values.size(), 0.);
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

size_t EpsilonGreedySampler::sample(const Vector &values, TransitionType &tt) const
{
  if (rand_->get() < epsilon_)
  {
    tt = ttExploratory;
    return rand_->getInteger(values.size());
  }

  return GreedySampler::sample(values, tt);
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

//////////////////////////////////////////

void EpsilonGreedyOUSampler::request(ConfigurationRequest *config)
{
  EpsilonGreedySampler::request(config);

  config->push_back(CRP("min", "vector.action_min", "Lower limit", min_, CRP::System));
  config->push_back(CRP("max", "vector.action_max", "Upper limit", max_, CRP::System));
  config->push_back(CRP("steps", "Discretization steps per dimension", steps_, CRP::Configuration));

  config->push_back(CRP("use_ou", "Use Ornstein-Uhlenbeck process", use_ou_, CRP::System, 0, 1));
  config->push_back(CRP("theta", "Theta parameter of Ornstein-Uhlenbeck", theta_, CRP::System, 0.0, DBL_MAX));
  config->push_back(CRP("sigma", "Sigma parameter of Ornstein-Uhlenbeck", sigma_, CRP::System, 0.0, DBL_MAX));
  config->push_back(CRP("center", "Centering parameter of Ornstein-Uhlenbeck", center_, CRP::Configuration));

  config->push_back(CRP("delta", "Delta of PADA", delta_, CRP::System, 0, INT_MAX));
}

void EpsilonGreedyOUSampler::configure(Configuration &config)
{
  EpsilonGreedySampler::configure(config);

  min_ = config["min"].v();
  max_ = config["max"].v();
  steps_ = config["steps"].v();

  use_ou_ = config["use_ou"];
  theta_ = config["theta"];
  sigma_ = config["sigma"];
  center_ = config["center"].v();

  delta_ = config["delta"];

  if (min_.size() != max_.size() || min_.size() != steps_.size())
    throw bad_param("sampler/epsilon_greedy_ou:{min,max,steps}");

  action_.resize(steps_.size());
  prev_action_.resize(steps_.size());
  for (int i = 0; i < steps_.size(); i++)
    action_[i] = prev_action_[i] = floor(steps_[i]/2);
}

void EpsilonGreedyOUSampler::reconfigure(const Configuration &config)
{
  EpsilonGreedySampler::reconfigure(config);
}

EpsilonGreedyOUSampler *EpsilonGreedyOUSampler::clone()
{
  EpsilonGreedyOUSampler *egs = new EpsilonGreedyOUSampler(*this);
  egs->rand_ = rand_->clone();

  return egs;
}

size_t EpsilonGreedyOUSampler::sample(const Vector &values, TransitionType &tt) const
{
  size_t mai = 0;

  if (rand_->get() < epsilon_)
  {
    tt = ttExploratory;
    if (use_ou_)
    {
      // Ornstein-Uhlenbeck process
      for (int i = 0; i < steps_.size(); i++)
      {
        action_[i] = prev_action_[i] + theta_ * (center_[i] - prev_action_[i])+ sigma_ * rand_->getNormal(0, 1);
        action_[i] = fmin(fmax(round(action_[i]), 0), steps_[i]-1);
      }
      mai = action_[0] + action_[1]*steps_[0] + action_[2]*steps_[0]*steps_[1];
      //std::cout << "Action exp: " << action_ << "; ii: " << mai << std::endl;
    }
    else
    {
      // random
      // PADA random action selection rule
      action_ << fmax(prev_action_[0]-delta_, 0), fmax(prev_action_[1]-delta_, 0), fmax(prev_action_[2]-delta_, 0);
      std::vector<size_t> iiv;
      for (int i = action_[0]; i <= fmin(prev_action_[0]+delta_, steps_[0]-1); i++)
      for (int j = action_[1]; j <= fmin(prev_action_[1]+delta_, steps_[1]-1); j++)
      for (int k = action_[2]; k <= fmin(prev_action_[2]+delta_, steps_[2]-1); k++)
      {
        size_t ii = i + j*steps_[0] + k*steps_[0]*steps_[1];
        iiv.push_back(ii);
      }
      // select random action out of limited possibilities
      int r = rand_->getInteger(iiv.size());
      mai = iiv[r];
    }
  }
  else
  {
    tt = ttGreedy;
    // PADA action selection rule
    int i0 = fmax(prev_action_[0]-delta_, 0);
    int j0 = fmax(prev_action_[1]-delta_, 0);
    int k0 = fmax(prev_action_[2]-delta_, 0);
    action_ << i0, j0, k0;
    for (int i = i0; i <= fmin(prev_action_[0]+delta_, steps_[0]-1); i++)
    for (int j = j0; j <= fmin(prev_action_[1]+delta_, steps_[1]-1); j++)
    for (int k = k0; k <= fmin(prev_action_[2]+delta_, steps_[2]-1); k++)
    {
      size_t ii = i + j*steps_[0] + k*steps_[0]*steps_[1];
      if (values[ii] > values[mai])
      {
        mai = ii;
        action_ << i, j, k;
      }
    }
  }
  prev_action_ = action_;
  return mai;
}

void EpsilonGreedyOUSampler::distribution(const Vector &values, Vector *distribution) const
{
  EpsilonGreedySampler::distribution(values, distribution);
}
