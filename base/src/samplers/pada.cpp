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

  state_idx_v_.resize(delta_.size());
  Vector initial_state = ConstantVector(delta_.size(), 0.0); // default action = 0
  discretizer_->discretize(initial_state, &state_idx_v_);

  env_event_ = (Signal*)config["contact_signal"].ptr();
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

void PADASampler::increment(IndexVector &idx_v, const IndexVector &lower_bound, const IndexVector &upper_bound) const
{
  for (int ii = 0; ii < lower_bound.size(); ii++)
  {
    if (idx_v[ii] < upper_bound[ii])
    {
      idx_v[ii]++;
      break;
    }
    else
      idx_v[ii] = lower_bound[ii];
  }
}

Vector PADASampler::env_event_processor() const
{
  Vector delta = delta_;
  Vector data;
  env_event_->get(&data);
  if (data.size())
  {
    if (data[0])
    {
      // contact happened
      int hipright, hipleft, kneeleft;
      hipright = data[2];
      hipleft  = data[3];
      kneeleft = data[4];

      TRACE(state_idx_v_);
      size_t tmp = state_idx_v_[hipright];
      state_idx_v_[hipright] = state_idx_v_[hipleft];
      state_idx_v_[hipleft] = tmp;

      const double prev_knee_auto_actuated = data[1];
      double nearest = DBL_MAX;
      for (int i = 0; i < discretizer_->steps()[kneeleft]; i++)
      {
        double v = (i-3)*(10.7/3);
        if (fabs(prev_knee_auto_actuated - nearest) > fabs(prev_knee_auto_actuated - v))
        {
          nearest = v;
          state_idx_v_[kneeleft] = i;
        }
      }
      TRACE(state_idx_v_);
    }

    if (data[5]) // start function => any action is possible
    {
      for (int ii = 0; ii < state_idx_v_.size(); ii++)
        delta[ii] = INT_MAX;
    }
  }

  //--------------------------------------
  // Uncomment for any signal for the knee
  // delta[0] = delta[1] = delta[2] = INT_MAX;
  //--------------------------------------

  return delta;
}

void PADASampler::get_bounds(Vector &delta, IndexVector &lower_bound, IndexVector &upper_bound) const
{
  // select indexes of upper and lower bounds
  lower_bound.resize(state_idx_v_.size());
  upper_bound.resize(state_idx_v_.size());
  for (int ii = 0; ii < state_idx_v_.size(); ii++)
  {
    lower_bound[ii] = fmax(state_idx_v_[ii]-delta[ii], 0);
    upper_bound[ii] = fmin(state_idx_v_[ii]+delta[ii], discretizer_->steps()[ii]-1);
  }
  TRACE(lower_bound);
  TRACE(upper_bound);
}

size_t PADASampler::exploration_step(IndexVector &lower_bound, IndexVector &upper_bound) const
{
  // collect all variants of discretized vectors within bounds
  IndexVector idx_v = lower_bound;
  std::vector<IndexVector> idx_collection;
  while (!std::equal(idx_v.begin(), idx_v.end(), upper_bound.begin()))
  {
    idx_collection.push_back(idx_v);
    TRACE(idx_v);
    increment(idx_v, lower_bound, upper_bound);
  }
  idx_collection.push_back(idx_v);
  TRACE(idx_v);

  // select random sample
  int r = rand_->getInteger(idx_collection.size());
  state_idx_v_ = idx_collection[r];
  TRACE(state_idx_v_);

  // convert to an index
  return discretizer_->convert(state_idx_v_);
}

size_t PADASampler::exploitation_step(const Vector &values, IndexVector &lower_bound, IndexVector &upper_bound) const
{
  IndexVector idx_v;
  size_t max_idx, loop_idx;

  // select the best value within bounds
  state_idx_v_ = idx_v = lower_bound;
  max_idx = discretizer_->convert(idx_v);
  increment(idx_v, lower_bound, upper_bound);
  TRACE(idx_v);
  while (!std::equal(idx_v.begin(), idx_v.end(), upper_bound.begin()))
  {
    loop_idx = discretizer_->convert(idx_v);
    if (values[loop_idx] > values[max_idx])
    {
      max_idx = loop_idx;
      state_idx_v_ = idx_v;
      TRACE(values[max_idx]);
    }
    increment(idx_v, lower_bound, upper_bound);
    TRACE(idx_v);
  }

  // Verification test: best action with no bounds
/*
  size_t greedy_idx = GreedySampler::sample(values, tt);
  if (greedy_idx != max_idx)
    std::cout << "Not correct action" << std::endl;
*/
  return max_idx;
}


size_t PADASampler::sample(const Vector &values, TransitionType &tt) const
{
  IndexVector lower_bound, upper_bound;
  Vector delta = env_event_processor();
  get_bounds(delta, lower_bound, upper_bound);

  if (rand_->get() < epsilon_)
  {
    tt = ttExploratory;
    return exploration_step(lower_bound, upper_bound);
  }
  else
  {
    tt = ttGreedy;
    return exploitation_step(values, lower_bound, upper_bound);
  }
}

size_t PADASampler::sample(const Vector &values, const IndexVector &state, TransitionType &tt) const
{
  state_idx_v_ = state;
  return sample(values, tt);
}

//////////////////////////////////////////////////////////

EpsilonPADASampler *EpsilonPADASampler::clone()
{
  EpsilonPADASampler *egs = new EpsilonPADASampler(*this);
  egs->rand_ = rand_->clone();
  return egs;
}

size_t EpsilonPADASampler::sample(const Vector &values, TransitionType &tt) const
{
  size_t idx;

  if (rand_->get() < epsilon_)
  {
    tt = ttExploratory;
    std::vector<size_t> lower_bound, upper_bound;
    Vector delta = env_event_processor();
    get_bounds(delta, lower_bound, upper_bound);
    idx = exploration_step(lower_bound, upper_bound);
  }
  else
  {
    tt = ttGreedy;
    idx = GreedySampler::sample(values, tt);
    state_idx_v_ = discretizer_->convert(idx);
    TRACE(state_idx_v_);
  }
/*
  size_t idx_test = discretizer_->convert(state_idx_v_);
  if (idx_test != idx)
    std::cout << "Not correct action" << std::endl;
*/
  return idx;
}

