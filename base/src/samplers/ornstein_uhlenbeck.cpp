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
  EpsilonGreedySampler::request(config);

  config->push_back(CRP("discretizer", "discretizer.action", "Action discretizer", discretizer_));
  config->push_back(CRP("theta", "Theta parameter of Ornstein-Uhlenbeck", theta_, CRP::Configuration));
  config->push_back(CRP("sigma", "Sigma parameter of Ornstein-Uhlenbeck", sigma_, CRP::Configuration));
  config->push_back(CRP("center", "Centering parameter of Ornstein-Uhlenbeck", center_, CRP::Configuration));
  config->push_back(CRP("contact_signal", "signal", "Signal", env_event_, true));

  config->push_back(CRP("memory", "vector/signal", "Pointer to the vector of the previous action", CRP::Provided));
}

void OrnsteinUhlenbeckSampler::configure(Configuration &config)
{
  EpsilonGreedySampler::configure(config);

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

  env_event_ = (VectorSignal*)config["contact_signal"].ptr();

  config.set("memory", noise_);
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

void OrnsteinUhlenbeckSampler::env_event_processor()
{
  LargeVector data = env_event_->get();

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

void OrnsteinUhlenbeckSampler::evolve_noise()
{
  TRACE(noise_);
  for (int i = 0; i < noise_.size(); i++)
    noise_[i] = noise_[i] + theta_[i] * (center_[i] - noise_[i])+ sigma_[i] * rand_->getNormal(0, 1);
  TRACE(noise_);
  memory_->set(noise_);
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
  // Greedy action selection
  size_t offset = GreedySampler::sample(values, tt);
  TRACE(discretizer_->at(offset));

  // Supress noise at the start of an episode and beginning of the step (?)
  env_event_processor();
  TRACE(noise_);

  // add noise to a signal
  evolve_noise();
  IndexVector state_idx;
  state_idx.resize(noise_.size());
  mix_signal_noise(discretizer_->at(offset), noise_, state_idx);

  // find value index of the sample and keep it for nex run
  offset = discretizer_->offset(state_idx);

  tt = ttExploratory;
  return offset;
}

//////////////////////////////////////////////////////////

void ACOrnsteinUhlenbeckSampler::configure(Configuration &config)
{
  OrnsteinUhlenbeckSampler::configure(config);

  IndexVector center_idx;
  center_idx.resize(center_.size());
  discretizer_->discretize(center_, &center_idx);
  offset_ = discretizer_->offset(center_idx);

  Vector smp_vec = discretizer_->at(offset_);
  config.set("memory", smp_vec);
}

ACOrnsteinUhlenbeckSampler *ACOrnsteinUhlenbeckSampler::clone()
{
  ACOrnsteinUhlenbeckSampler *egs = new ACOrnsteinUhlenbeckSampler(*this);
  egs->rand_ = rand_->clone();
  return egs;
}

size_t ACOrnsteinUhlenbeckSampler::sample(const LargeVector &values, TransitionType &tt)
{
  LargeVector data = env_event_->get();

  if (rand_->get() < epsilon_ && data[5] == 0) // start function (sig[5] == 1) => any action is possible
  {
    tt = ttExploratory;

    // convert array index to values
    Vector smp_vec = discretizer_->at(offset_);//state_variants_[offset_];

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

  Vector smp_vec = discretizer_->at(offset_);
  memory_->set(smp_vec);

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
  size_t offset = GreedySampler::sample(values, tt);
  TRACE(discretizer_->at(offset));

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
    IndexVector state_idx;
    state_idx.resize(noise_.size());
    mix_signal_noise(discretizer_->at(offset), noise_, state_idx);
    TRACE(state_idx);

    offset = discretizer_->offset(state_idx);
    TRACE(discretizer_->at(offset));
  }

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
  pada_.configure(config);

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
  pada_.set_offset(offset_);
  offset_ = pada_.sample(values, tt);
  TRACE(discretizer_->at(offset_));

  // Supress noise at the start of an episode
  env_event_processor();
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

  tt = ttExploratory;
  return offset_;
}

