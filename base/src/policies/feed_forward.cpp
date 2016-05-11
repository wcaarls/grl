/** \file feed_forward.cpp
 * \brief Feed forward policy source file.
 *
 * \author    Ivan Koryakovskiy <i.koryakovskiy@gmail.com>
 * \date      2016-02-11
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

#include <grl/policies/feed_forward.h>

using namespace grl;

REGISTER_CONFIGURABLE(FeedForwardPolicy)

void FeedForwardPolicy::request(ConfigurationRequest *config)
{
  config->push_back(CRP("action_dims", "Number of action dimensions", action_dims_, CRP::Configuration, 1));
  config->push_back(CRP("time_control", "Sequence of control vectors with a timestamp", time_control_));
  config->push_back(CRP("input", "string.input_", "CSV file with timestep and controls"));
}

void FeedForwardPolicy::configure(Configuration &config)
{
  action_dims_ = config["action_dims"];
  input_ = config["input"].str();
  shift_ = 1 + action_dims_; // time + action
  if (input_.empty())
  {
    time_control_ = config["time_control"].v();
  }
  else
  {
    std::ifstream file(input_);
    std::string line;
    std::vector<double> v;
    while( std::getline( file, line ) )
    {
      std::istringstream iss( line );
      std::string result;
      while( std::getline( iss, result, ',' ) )
         v.push_back(::atof(result.c_str()));
    }
    file.close();
    toVector(v, time_control_);
  }

  if (time_control_.size() % shift_ != 0)
    throw bad_param("policy/feed_forward:time_control");
}

void FeedForwardPolicy::reconfigure(const Configuration &config)
{

}

FeedForwardPolicy *FeedForwardPolicy::clone() const
{
  return new FeedForwardPolicy(*this);
}

TransitionType FeedForwardPolicy::act(double time, const Vector &in, Vector *out)
{
  if (time == 0)
    prev_time_idx_ = 0;

  // Search for the
  int ti = 0;
  while ((prev_time_idx_+ti)*shift_ < time_control_.size())
  {
    if (time_control_[(prev_time_idx_+ti)*shift_] <= time)
      ti++;
    else
      break;
  }
  int tis1 = (prev_time_idx_+ti-1)*shift_;
  int tis2 = (prev_time_idx_+ti  )*shift_;

  out->resize(action_dims_);
  // linear interpolation on the interval (t_tis1; t_tis2]
  double k = (time-time_control_[tis1])/(time_control_[tis2] - time_control_[tis1]);
  for (size_t ii=0; ii < action_dims_; ++ii)
    (*out)[ii] = (1-k) * time_control_[tis1+1+ii] +
                     k * time_control_[tis2+1+ii];

  prev_time_idx_ = ti-1;
  return ttGreedy;
}
