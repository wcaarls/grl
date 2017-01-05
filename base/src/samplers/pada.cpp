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
  config->push_back(CRP("pub_sub_pada_state", "signal/vector", "Publisher and subscriber to the value of action of the PADA familiy of samplers", pub_sub_pada_state_, true));
}

void PadaSampler::configure(Configuration &config)
{
  EpsilonGreedySampler::configure(config);

  discretizer_ = (Discretizer*)config["discretizer"].ptr();
  delta_ = config["delta"].v();
  sub_ic_signal_ = (VectorSignal*)config["sub_ic_signal"].ptr();
  pub_sub_pada_state_ = (VectorSignal*)config["pub_sub_pada_state"].ptr();

  for (int i = 0; i < delta_.size(); i++)
    if (delta_[i] < 0)
      throw bad_param("sampler/pada:delta");

  prev_action_ = ConstantVector(delta_.size(), 0.0); // default action = 0

  if (pub_sub_pada_state_)
    pub_sub_pada_state_->set(prev_action_);
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

size_t PadaSampler::sample(const LargeVector &values, TransitionType &tt)
{
  // offset is updated only if sampler_state is modified, i.e. contact happend for Leo
  if (pub_sub_pada_state_)
  {
    prev_action_ = pub_sub_pada_state_->get();
    CRAWL(prev_action_);
  }

  // bound possible actions
  LargeVector filtered;
  std::vector<size_t> idx;
  Vector delta = env_signal_processor();
  filter(delta, prev_action_, values, &filtered, &idx);

  size_t filtered_offset = EpsilonGreedySampler::sample(filtered, tt); // PadaSampler is derived from EpsilonGreedySampler => they share same epsilon
  CRAWL(filtered_offset);
  CRAWL(idx[filtered_offset]);

  prev_action_ = discretizer_->at(idx[filtered_offset]);
  CRAWL(prev_action_);

  if (pub_sub_pada_state_)
    pub_sub_pada_state_->set(prev_action_);

  return idx[filtered_offset];
}

/**
 * Returns both the Q values of the valid actions, and
 * an index array such that filtered[ii] = qvalues[idx[ii]]
 */
void PadaSampler::filter(const Vector &delta, const Vector &prev_out, const LargeVector &qvalues, LargeVector *filtered, std::vector<size_t> *idx) const
{
  if (prev_out.size() != delta.size())
    throw bad_param("sampler/pada:delta in filter");

  idx->clear();
  idx->reserve(qvalues.size());

  size_t aa=0;
  for (Discretizer::iterator it = discretizer_->begin(); it != discretizer_->end(); ++it, ++aa)
  {
    Vector action = *it;
    bool valid=true;
    for (size_t ii=0; ii < prev_out.size(); ++ii)
    {
      if (fabs(action[ii] - prev_out[ii]) > delta[ii])
      {
        valid=false;
        break;
      }
    }

    if (valid)
      idx->push_back(aa);
  }

  filtered->resize(idx->size());
  for (size_t ii=0; ii < idx->size(); ++ii)
    (*filtered)[ii] = qvalues[(*idx)[ii]];
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
  size_t offset;
  if (rand_->get() < epsilon_)
  {
    if (pub_sub_pada_state_)
    {
      // offset is updated only if sampler_state is modified, i.e. contact happend for Leo
      prev_action_ = pub_sub_pada_state_->get();
      TRACE(prev_action_);
    }

    // bound possible actions
    LargeVector filtered;
    std::vector<size_t> idx;
    Vector delta = env_signal_processor();
    filter(delta, prev_action_, values, &filtered, &idx);

    tt = ttExploratory;
    size_t filtered_offset = rand_->getInteger(filtered.size());
    TRACE(filtered_offset);
    offset = idx[filtered_offset];
    prev_action_ = discretizer_->at(offset);
  }
  else
  {
    tt = ttGreedy;
    offset = GreedySampler::sample(values, tt);
    prev_action_ = discretizer_->at(offset);
  }
  CRAWL(offset);
  CRAWL(prev_action_);
/*
  size_t idx_test = discretizer_->convert(state_idx_v_);
  if (idx_test != idx)
    std::cout << "Not correct action" << std::endl;
*/

  if (pub_sub_pada_state_)
    pub_sub_pada_state_->set(prev_action_);

  return offset;
}

