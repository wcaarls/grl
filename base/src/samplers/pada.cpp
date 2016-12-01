/** \file pada.h
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

REGISTER_CONFIGURABLE(PADASampler)
REGISTER_CONFIGURABLE(EpsilonPADASampler)

void PADASampler::request(ConfigurationRequest *config)
{
  EpsilonGreedySampler::request(config);

  config->push_back(CRP("discretizer", "discretizer.action", "Action discretizer", discretizer_));
  config->push_back(CRP("delta", "Delta of PADA", delta_, CRP::Configuration));
  config->push_back(CRP("contact_signal", "signal", "Signal", env_event_, true));
}

void PADASampler::configure(Configuration &config)
{
  EpsilonGreedySampler::configure(config);

  discretizer_ = (Discretizer*)config["discretizer"].ptr();
  delta_ = config["delta"].v();

  for (int i = 0; i < delta_.size(); i++)
    if (delta_[i] < 0)
      throw bad_param("sampler/pada:delta");

  Vector initial_state = ConstantVector(delta_.size(), 0.0); // default action = 0
  IndexVector state_idx;
  discretizer_->discretize(initial_state, &state_idx);
  offset_ = discretizer_->offset(state_idx);

  env_event_ = (VectorSignal*)config["contact_signal"].ptr();
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

Vector PADASampler::env_event_processor()
{
  Vector delta = delta_;
  LargeVector data = env_event_->get();
  if (data.size())
  {
    if (data[0])
    {
      // contact happened
      int hipright, hipleft, kneeleft;
      hipright = data[2];
      hipleft  = data[3];
      kneeleft = data[4];

      TRACE(offset_);
      IndexVector state_idx;
      discretizer_->at(offset_, &state_idx);
      size_t tmp = state_idx[hipright];
      state_idx[hipright] = state_idx[hipleft];
      state_idx[hipleft] = tmp;

      const double prev_knee_auto_actuated = data[1];
      double nearest = DBL_MAX;
      for (int i = 0; i < discretizer_->steps()[kneeleft]; i++)
      {
        double v = (i-3)*(10.7/3);
        if (fabs(prev_knee_auto_actuated - nearest) > fabs(prev_knee_auto_actuated - v))
        {
          nearest = v;
          state_idx[kneeleft] = i;
        }
      }
      offset_ = discretizer_->offset(state_idx);
      TRACE(offset_);
    }

    if (data[5]) // start function => any action is possible
    {
      for (int ii = 0; ii < delta.size(); ii++)
        delta[ii] = INT_MAX;
    }
  }

  //--------------------------------------
  // Uncomment for any signal for the knee
  // delta[0] = delta[1] = delta[2] = INT_MAX;
  //--------------------------------------

  return delta;
}

void PADASampler::get_bounds(size_t offset, Vector &delta, IndexVector &lower_bound, IndexVector &upper_bound) const
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

size_t PADASampler::exploration_step(Discretizer::bounded_iterator &bit) const
{
  IndexVector rnd_idx = IndexVector::Constant(delta_.size(), 0);
  for (int i = 0; i < delta_.size(); i++)
    rnd_idx[i] = round(rand_->getUniform(bit.lower_bound[i], bit.upper_bound[i]));
  TRACE(rnd_idx);

  return discretizer_->offset(rnd_idx);
}

size_t PADASampler::exploitation_step(const LargeVector &values, Discretizer::bounded_iterator &bit) const
{
  size_t loop_offset, max_offset = bit.offset();
  TRACE(*bit);
  while (bit != bit.end())
  {
    ++bit;
    TRACE(*bit);
    loop_offset = bit.offset();
    if (values[loop_offset] > values[max_offset])
    {
      max_offset = loop_offset;
      TRACE(values[max_offset]);
    }
  }
/*
  // Verification test: best action with no bounds
  TransitionType tt;
  size_t greedy_idx = GreedySampler::sample(values, tt);
  if (greedy_idx != max_idx)
    std::cout << "Not correct action" << std::endl;
*/
  return max_offset;
}

size_t PADASampler::sample(const LargeVector &values, TransitionType &tt)
{
  IndexVector lower_bound, upper_bound;
  Vector delta = env_event_processor();
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
  return offset_;
}

//////////////////////////////////////////////////////////

EpsilonPADASampler *EpsilonPADASampler::clone()
{
  EpsilonPADASampler *egs = new EpsilonPADASampler(*this);
  egs->rand_ = rand_->clone();
  return egs;
}

size_t EpsilonPADASampler::sample(const LargeVector &values, TransitionType &tt)
{
  if (rand_->get() < epsilon_)
  {
    tt = ttExploratory;
    IndexVector lower_bound, upper_bound;
    Vector delta = env_event_processor();
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
  return offset_;
}

