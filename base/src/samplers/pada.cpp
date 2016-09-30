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

  for (int i = 0; i < delta_.size(); i++)
    if (delta_[i] < 0)
      throw bad_param("sampler/pada:delta");

  for (int i = 0; i < steps_.size(); i++)
    sample_idx_.push_back((steps_[i]-1)/2); // initial action centered by default
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

void PADASampler::increment(std::vector<size_t> &idx, const std::vector<size_t> &lower_idx, const std::vector<size_t> &upper_idx) const
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
  std::vector<size_t> lower_idx, upper_idx, current_idx;
  lower_idx.resize(sample_idx_.size());
  upper_idx.resize(sample_idx_.size());
  for (int ii = 0; ii < sample_idx_.size(); ii++)
  {
    lower_idx[ii] = fmax(sample_idx_[ii]-delta_[ii], 0);
    upper_idx[ii] = fmin(sample_idx_[ii]+delta_[ii], steps_[ii]-1);
  }
  TRACE(lower_idx);
  TRACE(upper_idx);

  if (rand_->get() < epsilon_) // skip to Greedy if action is not initialized yet
  {
    tt = ttExploratory;

    // collect all variants of discretized vectors within bounds
    current_idx = lower_idx;
    std::vector<std::vector<size_t>> v_current_idx;
    while (!std::equal(current_idx.begin(), current_idx.end(), upper_idx.begin()))
    {
      v_current_idx.push_back(current_idx);
      TRACE(current_idx);
      increment(current_idx, lower_idx, upper_idx);
    }
    v_current_idx.push_back(current_idx);
    TRACE(current_idx);

    // select random sample
    int r = rand_->getInteger(v_current_idx.size());
    sample_idx_ = v_current_idx[r];
    TRACE(sample_idx_);

    // convert to an index
    size_t mai;
    discretizer_->convert(sample_idx_, mai);
    return mai;
  }
  else
  {
    tt = ttGreedy;
    size_t max_mai;

    // select the best value within bounds
    current_idx = lower_idx;
    discretizer_->convert(current_idx, max_mai);
    increment(current_idx, lower_idx, upper_idx);
    TRACE(current_idx);
    while (!std::equal(current_idx.begin(), current_idx.end(), upper_idx.begin()))
    {
      size_t mai;
      discretizer_->convert(current_idx, mai);
      if (values[mai] > values[max_mai])
      {
        max_mai = mai;
        sample_idx_ = current_idx;
        TRACE(values[max_mai]);
      }
      increment(current_idx, lower_idx, upper_idx);
      TRACE(current_idx);
    }
/*
    // Verification, best action for no bounds
    size_t mai = GreedySampler::sample(values, tt);
    if (mai != max_mai)
      std::cout << "Not correct action" << std::endl;
*/
    return max_mai;
  }
}
