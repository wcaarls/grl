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
REGISTER_CONFIGURABLE(OrnsteinUhlenbeckSampler2)
REGISTER_CONFIGURABLE(EpsilonOrnsteinUhlenbeckSampler)
REGISTER_CONFIGURABLE(PadaOrnsteinUhlenbeckSampler)

void OrnsteinUhlenbeckSampler::request(ConfigurationRequest *config)
{
  EpsilonGreedySampler::request(config);

  config->push_back(CRP("discretizer", "discretizer.action", "Action discretizer", discretizer_));
  config->push_back(CRP("theta", "Theta parameter of Ornstein-Uhlenbeck", theta_, CRP::Configuration));
  config->push_back(CRP("sigma", "Sigma parameter of Ornstein-Uhlenbeck", sigma_, CRP::Configuration));
  config->push_back(CRP("center", "Centering parameter of Ornstein-Uhlenbeck", center_, CRP::Configuration));
  config->push_back(CRP("contact_signal", "signal", "Signal", env_event_, true));
}

void OrnsteinUhlenbeckSampler::configure(Configuration &config)
{
  EpsilonGreedySampler::configure(config);

  discretizer_ = (Discretizer*)config["discretizer"].ptr();
  discretizer_->options(&state_variants_);

  theta_ = config["theta"].v();
  sigma_ = config["sigma"].v();
  center_ = config["center"].v();

  if (theta_.size() != sigma_.size() || sigma_.size() != center_.size() || center_.size() == 0)
    throw bad_param("sampler/ornstein_ohlenbeck:{theta,sigma,center}");

  IndexVector center_idx_v;
  center_idx_v.resize(center_.size());
  discretizer_->discretize(center_, &center_idx_v);
  state_idx_ = discretizer_->convert(center_idx_v);

  env_event_ = (Signal*)config["contact_signal"].ptr();
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
  Vector data;
  env_event_->get(&data);

  if (rand_->get() < epsilon_ && data[5] == 0) // start function (sig[5] == 1) => any action is possible
  {
    tt = ttExploratory;

    // convert array index to values
    Vector smp_vec = state_variants_[state_idx_];

    // mirror if needed
    if (data.size() && data[0])
    {
      int hipright, hipleft, kneeleft;
      hipright = data[2];
      hipleft  = data[3];
      kneeleft = data[4];

      TRACE(smp_vec);
      double tmp = smp_vec[hipright];
      smp_vec[hipright] = smp_vec[hipleft];
      smp_vec[hipleft] = tmp;

      smp_vec[kneeleft] = data[1];
    }
    if (data.size() && data[5])
      TRACE("Error: No OU at start");
    TRACE(smp_vec);

    // pertub action according to Ornstein-Uhlenbeck
    for (int i = 0; i < smp_vec.size(); i++)
      smp_vec[i] = smp_vec[i] + theta_[i] * (center_[i] - smp_vec[i])+ sigma_[i] * rand_->getNormal(0, 1);
    TRACE(smp_vec);

    // find nearest discretized sample (min-max bounds preserved automatically)
    IndexVector idx_v;
    idx_v.resize(center_.size());
    discretizer_->discretize(smp_vec, &idx_v);
    TRACE(smp_vec);
    TRACE(idx_v);

    // find value index of the sample and keep it for nex run
    state_idx_ = discretizer_->convert(idx_v);
  }
  else
    state_idx_ = GreedySampler::sample(values, tt);

  return state_idx_;
}

/////////////////////////////

OrnsteinUhlenbeckSampler2 *OrnsteinUhlenbeckSampler2::clone()
{
  OrnsteinUhlenbeckSampler2 *egs = new OrnsteinUhlenbeckSampler2(*this);
  egs->rand_ = rand_->clone();
  return egs;
}

void OrnsteinUhlenbeckSampler2::env_event_processor() const
{
  Vector data;
  env_event_->get(&data);

  // Calculate noise
  if (data.size())
  {
    // start function (sig[5] == 1) => noise is zero
    if (data[5])
      noise_ = center_;

    // mirror if needed
    if (data[0])
    {
      int hipright, hipleft, kneeleft;
      hipright = data[2];
      hipleft  = data[3];
      kneeleft = data[4];
      TRACE(noise_);
      double tmp = noise_[hipright];
      noise_[hipright] = noise_[hipleft];
      noise_[hipleft] = tmp;
      noise_[kneeleft] = 0;
      TRACE(noise_);
    }
  }
}

void OrnsteinUhlenbeckSampler2::evolve_noise() const
{
  TRACE(noise_);
  for (int i = 0; i < noise_.size(); i++)
    noise_[i] = noise_[i] + theta_[i] * (center_[i] - noise_[i])+ sigma_[i] * rand_->getNormal(0, 1);
  TRACE(noise_);
}

void OrnsteinUhlenbeckSampler2::mix_signal_noise(const Vector &in, const Vector &noise, IndexVector &out) const
{
  // convert array index to values
  TRACE(in);
  Vector vec = in + 10.7*noise;
  TRACE(vec);

  // find nearest discretized sample (min-max bounds preserved automatically)
  discretizer_->discretize(vec, &out);
  TRACE(vec);
  TRACE(out);
}

size_t OrnsteinUhlenbeckSampler2::sample(const Vector &values, TransitionType &tt) const
{
  // Greedy action selection
  size_t idx = GreedySampler::sample(values, tt);
  TRACE(state_variants_[idx]);

  // Supress noise at the start of an episode and beginning of the step (?)
  env_event_processor();
  TRACE(noise_);

  // add noise to to signal
  evolve_noise();
  IndexVector state_idx_v;
  state_idx_v.resize(noise_.size());
  mix_signal_noise(state_variants_[idx], noise_, state_idx_v);

  // find value index of the sample and keep it for nex run
  idx = discretizer_->convert(state_idx_v);

  tt = ttExploratory;
  return idx;
}

////////////////////////////////////////////////////

void EpsilonOrnsteinUhlenbeckSampler::request(ConfigurationRequest *config)
{
  OrnsteinUhlenbeckSampler2::request(config);
  config->push_back(CRP("epsilon", "Exploration rate", epsilon_, CRP::Online));
}

void EpsilonOrnsteinUhlenbeckSampler::configure(Configuration &config)
{
  OrnsteinUhlenbeckSampler2::configure(config);
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

size_t EpsilonOrnsteinUhlenbeckSampler::sample(const Vector &values, TransitionType &tt) const
{
  size_t idx = GreedySampler::sample(values, tt);
  TRACE(state_variants_[idx]);

  // Supress noise at the start of an episode
  env_event_processor();
  TRACE(noise_);

  // Take next noise value
  evolve_noise();
  TRACE(noise_);

  if (rand_->get() < epsilon_)
  {
    // add noise to to signal
    tt = ttExploratory;
    IndexVector state_idx_v;
    state_idx_v.resize(noise_.size());
    mix_signal_noise(state_variants_[idx], noise_, state_idx_v);
    TRACE(state_idx_v);

    idx = discretizer_->convert(state_idx_v);
    TRACE(state_variants_[idx]);
  }

  return idx;
}

///////////////////////////////////////////////////////

void PadaOrnsteinUhlenbeckSampler::request(ConfigurationRequest *config)
{
  OrnsteinUhlenbeckSampler2::request(config);
  pada_.request(config);
}

void PadaOrnsteinUhlenbeckSampler::configure(Configuration &config)
{
  OrnsteinUhlenbeckSampler2::configure(config);
  pada_.configure(config);

  int state_dims = discretizer_->steps().size();
  state_idx_v_.resize(state_dims);
  Vector initial_state = ConstantVector(state_dims, 0.0); // default action = 0
  discretizer_->discretize(initial_state, &state_idx_v_);
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

size_t PadaOrnsteinUhlenbeckSampler::sample(const Vector &values, TransitionType &tt) const
{
  size_t idx = pada_.sample(values, state_idx_v_, tt);
  TRACE(state_variants_[idx]);

  // Supress noise at the start of an episode
  env_event_processor();
  TRACE(noise_);

  // Take next noise value
  evolve_noise();
  TRACE(noise_);

  // add noise to to signal
  tt = ttExploratory;
  mix_signal_noise(state_variants_[idx], noise_, state_idx_v_);
  TRACE(state_idx_v_);

  idx = discretizer_->convert(state_idx_v_);
  TRACE(state_variants_[idx]);

  return idx;
}

