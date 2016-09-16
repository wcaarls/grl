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
REGISTER_CONFIGURABLE(OrnsteinUhlenbeckSampler)
REGISTER_CONFIGURABLE(PADASampler)

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

void OrnsteinUhlenbeckSampler::request(ConfigurationRequest *config)
{
  EpsilonGreedySampler::request(config);

  config->push_back(CRP("discretizer", "discretizer.action", "Action discretizer", discretizer_));
  config->push_back(CRP("theta", "Theta parameter of Ornstein-Uhlenbeck", theta_, CRP::Configuration));
  config->push_back(CRP("sigma", "Sigma parameter of Ornstein-Uhlenbeck", sigma_, CRP::Configuration));
  config->push_back(CRP("center", "Centering parameter of Ornstein-Uhlenbeck", center_, CRP::Configuration));
}

void OrnsteinUhlenbeckSampler::configure(Configuration &config)
{
  EpsilonGreedySampler::configure(config);

  discretizer_ = (Discretizer*)config["discretizer"].ptr();

  if (theta_.size() != sigma_.size() || sigma_.size() != center_.size() || center_.size() == 0)
    throw bad_param("sampler/ornstein_ohlenbeck:{theta,sigma,center}");

  theta_ = config["theta"].v();
  sigma_ = config["sigma"].v();
  center_ = config["center"].v();

  discretizer_->convert(center_, mai_);
}

void OrnsteinUhlenbeckSampler::reconfigure(const Configuration &config)
{
  EpsilonGreedySampler::reconfigure(config);
}

OrnsteinUhlenbeckSampler *OrnsteinUhlenbeckSampler::clone()
{
  OrnsteinUhlenbeckSampler *egs = new OrnsteinUhlenbeckSampler(*this);
  egs->rand_ = rand_->clone();
  return egs;
}

size_t OrnsteinUhlenbeckSampler::sample(const Vector &values, TransitionType &tt) const
{
  if (rand_->get() < epsilon_)
  {
    tt = ttExploratory;

    // convert array index to values
    Vector smp;
    discretizer_->convert(mai_, smp);

    // pertub action according to Ornstein-Uhlenbeck
    for (int i = 0; i < smp.size(); i++)
      smp[i] = smp[i] + theta_[i] * (center_[i] - smp[i])+ sigma_[i] * rand_->getNormal(0, 1);

    // find nearest discretized sample (min-max bounds preserved automatically)
    discretizer_->discretize(smp);

    // find value index of the sample and keep it for nex run
    discretizer_->convert(smp, mai_);
  }
  else
    mai_ = GreedySampler::sample(values, tt);

  return mai_;
}

/////////////////////////////////////////////////////////////////

void PADASampler::request(ConfigurationRequest *config)
{
  EpsilonGreedySampler::request(config);

  config->push_back(CRP("discretizer", "discretizer.action", "Action discretizer", discretizer_));
  config->push_back(CRP("steps", "Discretization steps per dimension", steps_, CRP::Configuration));
  config->push_back(CRP("delta", "Delta of PADA", delta_, CRP::Configuration));
}

void PADASampler::configure(Configuration &config)
{
  EpsilonGreedySampler::configure(config);

  discretizer_ = (Discretizer*)config["discretizer"].ptr();
  steps_ = config["steps"].v();
  delta_ = config["delta"].v();

  if (steps_.size() != delta_.size())
    throw bad_param("sampler/pada:{steps, delta}");
}

void PADASampler::reconfigure(const Configuration &config)
{
  EpsilonGreedySampler::reconfigure(config);
}

PADASampler *PADASampler::clone()
{
  PADASampler *egs = new PADASampler(*this);
  egs->rand_ = rand_->clone();
  return egs;
}

void PADASampler::increment(Vector &idx, const Vector &lower_idx, Vector &upper_idx) const
{
  for (int ii = 0; ii < lower_idx.size(); ii++)
  {
    if (idx[ii] < upper_idx[ii])
    {
      idx[ii]++;
      break;
    }
    else
      idx[ii] = lower_idx[ii];
  }
}

size_t PADASampler::sample(const Vector &values, TransitionType &tt) const
{
  // select indexes of upper and lower bounds
  Vector lower_idx, upper_idx, current_idx;
  for (int ii = 0; ii < sample_idx_.size(); ii++)
  {
    lower_idx[ii] = fmax(sample_idx_[ii]-delta_[ii], 0);
    upper_idx[ii] = fmax(sample_idx_[ii]+delta_[ii], steps_[ii]-1);
  }

  if (rand_->get() < epsilon_ && sample_idx_.size() != 0) // skip to Greedy if action is not initialized yet
  {
    tt = ttExploratory;

    // collect all variants of discretized vectors withing bounds
    current_idx = lower_idx;
    std::vector<Vector> v_current_idx;
    while (current_idx != upper_idx)
    {
      v_current_idx.push_back(current_idx);
      increment(current_idx, lower_idx, upper_idx);
    }
    v_current_idx.push_back(current_idx);

    // select random sample
    int r = rand_->getInteger(v_current_idx.size());
    sample_idx_ = v_current_idx[r];

    // convert to an index
    size_t mai;
    discretizer_->convert_idx(sample_idx_, mai);
    return mai;
  }
  else
  {
    tt = ttGreedy;
    size_t max_mai;

    // select the best value within bounds
    sample_idx_ = current_idx;
    discretizer_->convert_idx(current_idx, max_mai);
    while (current_idx != upper_idx)
    {
      size_t mai;
      discretizer_->convert_idx(current_idx, mai);
      if (values[mai] > values[max_mai])
      {
        max_mai = mai;
        sample_idx_ = current_idx;
      }
      increment(current_idx, lower_idx, upper_idx);
    }
    return max_mai;
  }
}
