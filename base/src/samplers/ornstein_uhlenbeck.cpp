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
  config->push_back(CRP("pub_sub_ou_state", "signal/vector", "Publisher and subscriber to the value of noise (or action in the ACOU case) of the Ornstein Uhlenbeck familiy of samplers", pub_sub_ou_state_, true));
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
  pub_sub_ou_state_ = (VectorSignal*)config["pub_sub_ou_state"].ptr();

  noise_ = center_;
  if (pub_sub_ou_state_)
    pub_sub_ou_state_->set(noise_);
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
    Vector signal = sub_ic_signal_->get();
    if (signal[0] == sigEnvInit)
      noise_ = center_;
  }
}

void OrnsteinUhlenbeckSampler::evolve_noise()
{
  CRAWL(noise_);
  for (int i = 0; i < noise_.size(); i++)
    noise_[i] = noise_[i] + theta_[i] * (center_[i] - noise_[i])+ sigma_[i] * rand_->getNormal(0, 1);
  CRAWL(noise_);
}

Vector OrnsteinUhlenbeckSampler::mix_signal_noise(const Vector &in, const Vector &noise) const
{
  return in + noise_scale_*noise;
}

size_t OrnsteinUhlenbeckSampler::sample(double time, const LargeVector &values, TransitionType &tt)
{
  if (pub_sub_ou_state_)
    noise_ = pub_sub_ou_state_->get();

  // Greedy action selection
  size_t offset = GreedySampler::sample(values, tt);
  CRAWL(discretizer_->at(offset));

  // Supress noise at the start of an episode and beginning of the step (?)
  env_signal_processor();
  CRAWL(noise_);

  // add noise to a signal
  evolve_noise();
  Vector action_new = mix_signal_noise(discretizer_->at(offset), noise_);
  offset = discretizer_->discretize(&action_new);

  if (pub_sub_ou_state_)
    pub_sub_ou_state_->set(noise_);

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

  offset_ = discretizer_->discretize(&center_);

  if (pub_sub_ou_state_)
    pub_sub_ou_state_->set(center_);
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

size_t ACOrnsteinUhlenbeckSampler::sample(double time, const LargeVector &values, TransitionType &tt)
{
  LargeVector signal = sub_ic_signal_->get();

  if (rand_->get() < epsilon_ && signal[0] != sigEnvInit)
  {
    tt = ttExploratory;

    // read previous action values
    Vector smp_vec;
    if (pub_sub_ou_state_)
      smp_vec = pub_sub_ou_state_->get();
    else
      smp_vec = discretizer_->at(offset_);
    TRACE(smp_vec);

    // pertub action according to Ornstein-Uhlenbeck
    for (int i = 0; i < smp_vec.size(); i++)
      smp_vec[i] = smp_vec[i] + theta_[i] * (center_[i] - smp_vec[i])+ sigma_[i] * rand_->getNormal(0, 1);
    TRACE(smp_vec);

    offset_ = discretizer_->discretize(&smp_vec);
    TRACE(offset_);
  }
  else
    offset_ = GreedySampler::sample(values, tt);

  if (pub_sub_ou_state_)
    pub_sub_ou_state_->set(discretizer_->at(offset_));

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

size_t EpsilonOrnsteinUhlenbeckSampler::sample(double time, const LargeVector &values, TransitionType &tt)
{
  if (pub_sub_ou_state_)
    noise_ = pub_sub_ou_state_->get();

  size_t offset = GreedySampler::sample(values, tt);
  CRAWL(discretizer_->at(offset));

  // Supress noise at the start of an episode
  env_signal_processor();
  CRAWL(noise_);

  // Take next noise value
  evolve_noise();
  CRAWL(noise_);

  if (rand_->get() < epsilon_)
  {
    // add noise to to signal
    tt = ttExploratory;
    Vector action_new = mix_signal_noise(discretizer_->at(offset), noise_);
    TRACE(action_new);

    offset = discretizer_->discretize(&action_new);
  }

  if (pub_sub_ou_state_)
    pub_sub_ou_state_->set(noise_);

  return offset;
}

//////////////////////////////////////////////////////////

void PadaOrnsteinUhlenbeckSampler::request(ConfigurationRequest *config)
{
  OrnsteinUhlenbeckSampler::request(config);
  config->push_back(CRP("pada", "sampler", "Pada sampler", pada_));
  config->push_back(CRP("pub_new_action", "signal/vector", "Publisher of the signal with noise", pub_new_action_));
}

void PadaOrnsteinUhlenbeckSampler::configure(Configuration &config)
{
  OrnsteinUhlenbeckSampler::configure(config);
  pada_ = (Sampler*)config["pada"].ptr();
  pub_new_action_ = (VectorSignal*)config["pub_new_action"].ptr();
}

void PadaOrnsteinUhlenbeckSampler::reconfigure(const Configuration &config)
{
  pada_->reconfigure(config);
}

PadaOrnsteinUhlenbeckSampler *PadaOrnsteinUhlenbeckSampler::clone()
{
  PadaOrnsteinUhlenbeckSampler *egs = new PadaOrnsteinUhlenbeckSampler(*this);
  egs->rand_ = rand_->clone();
  return egs;
}

size_t PadaOrnsteinUhlenbeckSampler::sample(double time, const LargeVector &values, TransitionType &tt)
{
  if (pub_sub_ou_state_)
    noise_ = pub_sub_ou_state_->get();

  size_t offset = pada_->sample(values, tt);
  CRAWL(discretizer_->at(offset));

  // Supress noise at the start of an episode
  env_signal_processor();
  CRAWL(noise_);

  // Take next noise value
  evolve_noise();
  CRAWL(noise_);

  // add noise to to signal
  Vector action_new = mix_signal_noise(discretizer_->at(offset), noise_);
  CRAWL(action_new);

  /////////////////
  // Testing
  size_t offset_orig = offset;
  offset = discretizer_->discretize(&action_new);
  if (offset_orig != offset)
  {
    TRACE("OU noise affected PADA choise. This is correct if it happens once in a while.");
    TRACE(discretizer_->at(offset_orig));
    TRACE(discretizer_->at(offset));
  }
  /////////////////

  offset = discretizer_->discretize(&action_new);
  CRAWL(offset);

  // Inform pada sampler about the previous action (discretized)
  pub_new_action_->set(discretizer_->at(offset));

  if (pub_sub_ou_state_)
     pub_sub_ou_state_->set(noise_);

  tt = ttExploratory;
  return offset;
}

