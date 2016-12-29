/** \file ornstein_uhlenbeck.h
 * \brief Ornstein-Uhlenbeck samplers source file.
 *
 * \author    Ivan Koryakovskiy <i.koryakovskiy@tudelft.nl>
 * \date      2016-09-30
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
#include <grl/grl.h>
#include <grl/samplers/ornstein_uhlenbeck.h>

using namespace grl;

REGISTER_CONFIGURABLE(OrnsteinUhlenbeckSampler)
REGISTER_CONFIGURABLE(ACOrnsteinUhlenbeckSampler)
REGISTER_CONFIGURABLE(EpsilonOrnsteinUhlenbeckSampler)
REGISTER_CONFIGURABLE(PadaOrnsteinUhlenbeckSampler)

void OrnsteinUhlenbeckSampler::request(ConfigurationRequest *config)
{
  GreedySampler::request(config);

  config->push_back(CRP("discretizer", "discretizer.action", "Action discretizer", discretizer_));
  config->push_back(CRP("theta", "Theta parameter of Ornstein-Uhlenbeck", theta_, CRP::Configuration));
  config->push_back(CRP("sigma", "Sigma parameter of Ornstein-Uhlenbeck", sigma_, CRP::Configuration));
  config->push_back(CRP("center", "Centering parameter of Ornstein-Uhlenbeck", center_, CRP::Configuration));
  config->push_back(CRP("sub_ic_signal", "signal/vector", "Subscriber to the initialization and contact signal from environment", sub_ic_signal_, true));
  config->push_back(CRP("pub_sub_sampler_state", "signal/vector", "Publisher and subscriber of the sampler state with memory such as previous action, noise, etc.", pub_sub_sampler_state_, true));

}

void OrnsteinUhlenbeckSampler::configure(Configuration &config)
{
  GreedySampler::configure(config);

  discretizer_ = (Discretizer*)config["discretizer"].ptr();

  theta_ = config["theta"].v();
  sigma_ = config["sigma"].v();
  center_ = config["center"].v();

  if (theta_.size() != sigma_.size() || sigma_.size() != center_.size() || center_.size() == 0)
    throw bad_param("sampler/ornstein_ohlenbeck:{theta,sigma,center}");

  Vector neg_part = discretizer_->at(discretizer_->size()-1) - center_;
  Vector pos_part = center_ - discretizer_->at(0);

  noise_scale_.resize(center_.size());
  for (int i = 0; i < center_.size(); i++)
    noise_scale_[i] = std::max(pos_part[i], neg_part[i]);

  sub_ic_signal_ = (VectorSignal*)config["sub_ic_signal"].ptr();
  pub_sub_sampler_state_ = (VectorSignal*)config["pub_sub_sampler_state"].ptr();

  noise_ = center_;
  if (pub_sub_sampler_state_)
    pub_sub_sampler_state_->set(noise_);
}

void OrnsteinUhlenbeckSampler::reconfigure(const Configuration &config)
{
  GreedySampler::reconfigure(config);
}

OrnsteinUhlenbeckSampler *OrnsteinUhlenbeckSampler::clone()
{
  OrnsteinUhlenbeckSampler *egs = new OrnsteinUhlenbeckSampler(*this);
  egs->rand_ = rand_->clone();
  return egs;
}

void OrnsteinUhlenbeckSampler::env_signal_processor()
{
  if (sub_ic_signal_)
  {
    LargeVector signal = sub_ic_signal_->get();
    if (signal[0] == sigEnvInit)
      noise_ = center_;
  }
}

void OrnsteinUhlenbeckSampler::evolve_noise()
{
  TRACE(noise_);
  for (int i = 0; i < noise_.size(); i++)
    noise_[i] = noise_[i] + theta_[i] * (center_[i] - noise_[i])+ sigma_[i] * rand_->getNormal(0, 1);
  TRACE(noise_);
}

void OrnsteinUhlenbeckSampler::mix_signal_noise(const Vector &in, const Vector &noise, IndexVector &out) const
{
  // convert array index to values
  TRACE(in);
  Vector vec = in + noise_scale_*noise; // coefficient-wise product and summation
  TRACE(vec);

  // find nearest discretized sample (min-max bounds preserved automatically)
  discretizer_->discretize(vec, &out);
  TRACE(vec);
  TRACE(out);
}

size_t OrnsteinUhlenbeckSampler::sample(const LargeVector &values, TransitionType &tt)
{
  if (pub_sub_sampler_state_)
    noise_ = pub_sub_sampler_state_->get();

  // Greedy action selection
  size_t offset = GreedySampler::sample(values, tt);
  TRACE(discretizer_->at(offset));

  // Supress noise at the start of an episode and beginning of the step (?)
  env_signal_processor();
  TRACE(noise_);

  // add noise to a signal
  evolve_noise();
  IndexVector state_idx;
  state_idx.resize(noise_.size());
  mix_signal_noise(discretizer_->at(offset), noise_, state_idx);

  // find value index of the sample and keep it for nex run
  offset = discretizer_->offset(state_idx);

  if (pub_sub_sampler_state_)
    pub_sub_sampler_state_->set(noise_);

  tt = ttExploratory;
  return offset;
}

//////////////////////////////////////////////////////////
void ACOrnsteinUhlenbeckSampler::request(ConfigurationRequest *config)
{
  OrnsteinUhlenbeckSampler::request(config);
  config->push_back(CRP("epsilon", "Exploration rate", epsilon_, CRP::Online));
}

void ACOrnsteinUhlenbeckSampler::configure(Configuration &config)
{
  OrnsteinUhlenbeckSampler::configure(config);
  epsilon_ = config["epsilon"];

  IndexVector center_idx;
  center_idx.resize(center_.size());
  discretizer_->discretize(center_, &center_idx);
  offset_ = discretizer_->offset(center_idx);

  if (pub_sub_sampler_state_)
    pub_sub_sampler_state_->set(center_);
}

void ACOrnsteinUhlenbeckSampler::reconfigure(const Configuration &config)
{
  config.get("epsilon", epsilon_);
}

ACOrnsteinUhlenbeckSampler *ACOrnsteinUhlenbeckSampler::clone()
{
  ACOrnsteinUhlenbeckSampler *egs = new ACOrnsteinUhlenbeckSampler(*this);
  egs->rand_ = rand_->clone();
  return egs;
}

size_t ACOrnsteinUhlenbeckSampler::sample(const LargeVector &values, TransitionType &tt)
{
  LargeVector signal = sub_ic_signal_->get();

  if (rand_->get() < epsilon_ && signal[0] != sigEnvInit)
  {
    tt = ttExploratory;

    // read previous action values
    Vector smp_vec;
    if (pub_sub_sampler_state_)
      smp_vec = pub_sub_sampler_state_->get();
    else
      smp_vec = discretizer_->at(offset_);
    TRACE(smp_vec);

    // pertub action according to Ornstein-Uhlenbeck
    for (int i = 0; i < smp_vec.size(); i++)
      smp_vec[i] = smp_vec[i] + theta_[i] * (center_[i] - smp_vec[i])+ sigma_[i] * rand_->getNormal(0, 1);
    TRACE(smp_vec);

    // find nearest discretized sample (min-max bounds preserved automatically)
    IndexVector state_idx;
    state_idx.resize(center_.size());
    discretizer_->discretize(smp_vec, &state_idx);
    TRACE(smp_vec);
    TRACE(state_idx);

    // find value index of the sample and keep it for nex run
    offset_ = discretizer_->offset(state_idx);
  }
  else
    offset_ = GreedySampler::sample(values, tt);

  if (pub_sub_sampler_state_)
  {
    Vector smp_vec = discretizer_->at(offset_);
    pub_sub_sampler_state_->set(smp_vec);
  }

  return offset_;
}

//////////////////////////////////////////////////////////

void EpsilonOrnsteinUhlenbeckSampler::request(ConfigurationRequest *config)
{
  OrnsteinUhlenbeckSampler::request(config);
  config->push_back(CRP("epsilon", "Exploration rate", epsilon_, CRP::Online));
}

void EpsilonOrnsteinUhlenbeckSampler::configure(Configuration &config)
{
  OrnsteinUhlenbeckSampler::configure(config);
  epsilon_ = config["epsilon"];
}

void EpsilonOrnsteinUhlenbeckSampler::reconfigure(const Configuration &config)
{
  config.get("epsilon", epsilon_);
}

EpsilonOrnsteinUhlenbeckSampler *EpsilonOrnsteinUhlenbeckSampler::clone()
{
  EpsilonOrnsteinUhlenbeckSampler *egs = new EpsilonOrnsteinUhlenbeckSampler(*this);
  egs->rand_ = rand_->clone();
  return egs;
}

size_t EpsilonOrnsteinUhlenbeckSampler::sample(const LargeVector &values, TransitionType &tt)
{
  if (pub_sub_sampler_state_)
    noise_ = pub_sub_sampler_state_->get();

  size_t offset = GreedySampler::sample(values, tt);
  TRACE(discretizer_->at(offset));

  // Supress noise at the start of an episode
  env_signal_processor();
  TRACE(noise_);

  // Take next noise value
  evolve_noise();
  TRACE(noise_);

  if (rand_->get() < epsilon_)
  {
    // add noise to to signal
    tt = ttExploratory;
    IndexVector state_idx;
    state_idx.resize(noise_.size());
    mix_signal_noise(discretizer_->at(offset), noise_, state_idx);
    TRACE(state_idx);

    offset = discretizer_->offset(state_idx);
    TRACE(discretizer_->at(offset));
  }

  if (pub_sub_sampler_state_)
    pub_sub_sampler_state_->set(noise_);

  return offset;
}

//////////////////////////////////////////////////////////

void PadaOrnsteinUhlenbeckSampler::request(ConfigurationRequest *config)
{
  OrnsteinUhlenbeckSampler::request(config);
  pada_.request(config);
}

void PadaOrnsteinUhlenbeckSampler::configure(Configuration &config)
{
  OrnsteinUhlenbeckSampler::configure(config);

  // Since we prescribe the offset, the default implementation of PADA should work
  // (pub_sub_sampler_state_ should not be used inside)
  Configuration config_copy = config;
  config_copy.set("pub_sub_sampler_state", NULL);
  pada_.configure(config_copy);

  IndexVector center_idx;
  center_idx.resize(center_.size());
  discretizer_->discretize(center_, &center_idx);
  offset_ = discretizer_->offset(center_idx);
}

void PadaOrnsteinUhlenbeckSampler::reconfigure(const Configuration &config)
{
  pada_.reconfigure(config);
}

PadaOrnsteinUhlenbeckSampler *PadaOrnsteinUhlenbeckSampler::clone()
{
  PadaOrnsteinUhlenbeckSampler *egs = new PadaOrnsteinUhlenbeckSampler(*this);
  egs->rand_ = rand_->clone();
  return egs;
}

size_t PadaOrnsteinUhlenbeckSampler::sample(const LargeVector &values, TransitionType &tt)
{
  if (pub_sub_sampler_state_)
    noise_ = pub_sub_sampler_state_->get();

  pada_.set_offset(offset_);
  offset_ = pada_.sample(values, tt);
  TRACE(discretizer_->at(offset_));

  // Supress noise at the start of an episode
  env_signal_processor();
  TRACE(noise_);

  // Take next noise value
  evolve_noise();
  TRACE(noise_);

  // add noise to to signal
  IndexVector state_idx;
  mix_signal_noise(discretizer_->at(offset_), noise_, state_idx);
  TRACE(state_idx);

  offset_ = discretizer_->offset(state_idx);
  TRACE(discretizer_->at(offset_));

  if (pub_sub_sampler_state_)
     pub_sub_sampler_state_->set(noise_);

  tt = ttExploratory;
  return offset_;
}

