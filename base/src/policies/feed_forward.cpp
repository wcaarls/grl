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
#include <sys/stat.h>

using namespace grl;

REGISTER_CONFIGURABLE(FeedForwardPolicy)

void FeedForwardPolicy::request(ConfigurationRequest *config)
{
  config->push_back(CRP("input", "CSV file with timestep and controls", "", CRP::Configuration));
}

void FeedForwardPolicy::configure(Configuration &config)
{
  input_ = config["input"].str();

  std::vector<double> v;
  struct stat buffer;
  if (stat(input_.c_str(), &buffer) == 0)
  {
    std::ifstream file(input_);
    std::string line;
    bool first = true;
    while( std::getline( file, line ) )
    {
      std::istringstream iss( line );
      std::string result;
      while( std::getline( iss, result, ',' ) )
         v.push_back(::atof(result.c_str()));

      if (first)
      {
        shift_ = v.size();
        first = false;
      }

      if (time_control_.size() % shift_ != 0)
        throw bad_param("policy/feed_forward:time_control");
    }

    file.close();
  }
  else
  {
    INFO("Reading input as a vector of doubles");
    const std::vector<std::string> list = cutLongStr(input_);
    if (list.size() < 2)
      throw bad_param("policy/feed_forward:input");
    for (int i = 0; i < list.size(); i++)
      v.push_back(std::stod(list[i]));
  }

  toVector(v, time_control_);
}

void FeedForwardPolicy::reconfigure(const Configuration &config)
{

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

  out->resize(shift_-1); // do not count time

  double k;
  if (tis2 >= time_control_.size())
  {
    // keep the last active control if control for this time interval is not specified
    tis2 = time_control_.size() - shift_;
    tis1 = tis2 - shift_;
    k = 1;
    WARNING("Control for this time interval is not specified. Use last control of " << time_control_.block(0, tis2+1, 1, shift_-1));
  }
  else
  {
    // linear interpolation on the interval (t_tis1; t_tis2]
    k = (time-time_control_[tis1])/(time_control_[tis2] - time_control_[tis1]);
  }

  for (size_t ii=0; ii < shift_-1; ++ii)
    (*out)[ii] = (1-k) * time_control_[tis1+1+ii] +
                     k * time_control_[tis2+1+ii];

  prev_time_idx_ = ti-1;
  return ttGreedy;
}

