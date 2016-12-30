/** \file pada.cpp
 * \brief PADA samplers source file.
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
#include <grl/samplers/pada.h>

using namespace grl;

REGISTER_CONFIGURABLE(PadaSampler)
REGISTER_CONFIGURABLE(EpsilonPadaSampler)

void PadaSampler::request(ConfigurationRequest *config)
{
  EpsilonGreedySampler::request(config);

  config->push_back(CRP("discretizer", "discretizer.action", "Action discretizer", discretizer_));
  config->push_back(CRP("delta", "Delta of PADA", delta_, CRP::Configuration));
  config->push_back(CRP("sub_ic_signal", "signal/vector", "Subscriber to the initialization and contact signal from environment", sub_ic_signal_, true));
  config->push_back(CRP("pub_sub_pada", "signal/vector", "Publisher and subscriber to the value of action of the PADA familiy of samplers", pub_sub_sampler_state_, true));
}

void PadaSampler::configure(Configuration &config)
{
  EpsilonGreedySampler::configure(config);

  discretizer_ = (Discretizer*)config["discretizer"].ptr();
  delta_ = config["delta"].v();
  sub_ic_signal_ = (VectorSignal*)config["sub_ic_signal"].ptr();
  pub_sub_sampler_state_ = (VectorSignal*)config["pub_sub_pada"].ptr();

  for (int i = 0; i < delta_.size(); i++)
    if (delta_[i] < 0)
      throw bad_param("sampler/pada:delta");

  Vector initial_state = ConstantVector(delta_.size(), 0.0); // default action = 0
  IndexVector state_idx;
  discretizer_->discretize(initial_state, &state_idx);
  offset_ = discretizer_->offset(state_idx);

  if (pub_sub_sampler_state_)
    pub_sub_sampler_state_->set(initial_state);
}

void PadaSampler::reconfigure(const Configuration &config)
{
  EpsilonGreedySampler::reconfigure(config);
}

PadaSampler *PadaSampler::clone()
{
  PadaSampler *egs = new PadaSampler(*this);
  egs->rand_ = rand_->clone();
  return egs;
}

Vector PadaSampler::env_signal_processor()
{
  Vector delta = delta_;

  if (sub_ic_signal_)
  {
    Vector signal = sub_ic_signal_->get();
    if (signal[0] == sigEnvInit)
    {
      for (int ii = 0; ii < delta.size(); ii++)
        delta[ii] = INT_MAX;
    }
  }

  //--------------------------------------
  // Uncomment for any signal for the knee (debug)
  // delta[0] = delta[1] = delta[2] = INT_MAX;
  //--------------------------------------

  return delta;
}

void PadaSampler::get_bounds(size_t offset, Vector &delta, IndexVector &lower_bound, IndexVector &upper_bound) const
{
  // select indexes of upper and lower bounds
  IndexVector state_idx;
  discretizer_->at(offset, &state_idx);
  lower_bound.resize(state_idx.size());
  upper_bound.resize(state_idx.size());
  for (int ii = 0; ii < state_idx.size(); ii++)
  {
    lower_bound[ii] = fmax(state_idx[ii]-delta[ii], 0);
    upper_bound[ii] = fmin(state_idx[ii]+delta[ii], discretizer_->steps()[ii]-1);
  }
  TRACE(lower_bound);
  TRACE(upper_bound);
}

size_t PadaSampler::exploration_step(Discretizer::bounded_iterator &bit) const
{
  IndexVector rnd_idx = IndexVector::Constant(delta_.size(), 0);
  for (int i = 0; i < delta_.size(); i++)
    rnd_idx[i] = round(rand_->getUniform(bit.lower_bound[i], bit.upper_bound[i]));
  TRACE(rnd_idx);

  return discretizer_->offset(rnd_idx);
}

size_t PadaSampler::exploitation_step(const LargeVector &values, Discretizer::bounded_iterator &bit) const
{
  size_t loop_offset, max_offset = bit.offset();
  CRAWL(*bit);
  while (bit != bit.end())
  {
    ++bit;
    CRAWL(*bit);
    loop_offset = bit.offset();
    if (values[loop_offset] > values[max_offset])
    {
      max_offset = loop_offset;
      CRAWL(values[max_offset]);
    }
  }
/*
  // Verification test: best action with no bounds
  TransitionType tt;
  GreedySampler gs;
  size_t greedy_offset = const_cast<GreedySampler*>(&gs)->sample(values, tt);
  if (greedy_offset != max_offset)
    std::cout << "Not correct action" << std::endl;
*/
  return max_offset;
}

size_t PadaSampler::sample(const LargeVector &values, TransitionType &tt)
{
  if (pub_sub_sampler_state_)
  {
    // offset is updated only if sampler_state is modified, i.e. contact happend for Leo
    IndexVector action_idx;
    Vector action = pub_sub_sampler_state_->get();
    TRACE(action);
    discretizer_->discretize(action, &action_idx);
    TRACE(action_idx);
    offset_ = discretizer_->offset(action_idx);
  }

  IndexVector lower_bound, upper_bound;
  Vector delta = env_signal_processor();
  get_bounds(offset_, delta, lower_bound, upper_bound);

  Discretizer::bounded_iterator bit = Discretizer::bounded_iterator(discretizer_, Vector(), lower_bound, lower_bound, upper_bound);
  if (rand_->get() < epsilon_)
  {
    tt = ttExploratory;
    offset_ = exploration_step(bit);
  }
  else
  {
    tt = ttGreedy;
    offset_ = exploitation_step(values, bit);
  }

  if (pub_sub_sampler_state_)
    pub_sub_sampler_state_->set(discretizer_->at(offset_));

  return offset_;
}

//////////////////////////////////////////////////////////

EpsilonPadaSampler *EpsilonPadaSampler::clone()
{
  EpsilonPadaSampler *egs = new EpsilonPadaSampler(*this);
  egs->rand_ = rand_->clone();
  return egs;
}

size_t EpsilonPadaSampler::sample(const LargeVector &values, TransitionType &tt)
{
  if (rand_->get() < epsilon_)
  {
    if (pub_sub_sampler_state_)
    {
      // offset is updated only if sampler_state is modified, i.e. contact happend for Leo
      IndexVector action_idx;
      Vector action = pub_sub_sampler_state_->get();
      TRACE(action);
      discretizer_->discretize(action, &action_idx);
      TRACE(action_idx);
      offset_ = discretizer_->offset(action_idx);
    }

    tt = ttExploratory;
    IndexVector lower_bound, upper_bound;
    Vector delta = env_signal_processor();
    get_bounds(offset_, delta, lower_bound, upper_bound);
    Discretizer::bounded_iterator bit = Discretizer::bounded_iterator(discretizer_, Vector(), lower_bound, lower_bound, upper_bound);
    offset_ = exploration_step(bit);
  }
  else
  {
    tt = ttGreedy;
    offset_ = GreedySampler::sample(values, tt);
  }
  TRACE(offset_);
/*
  size_t idx_test = discretizer_->convert(state_idx_v_);
  if (idx_test != idx)
    std::cout << "Not correct action" << std::endl;
*/

  Vector smp_vec = discretizer_->at(offset_);
  if (pub_sub_sampler_state_)
    pub_sub_sampler_state_->set(smp_vec);

  return offset_;
}

