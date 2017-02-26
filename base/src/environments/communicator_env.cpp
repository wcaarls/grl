/** \file Communicator_env.cpp
 * \brief Communicator environment source file.
 *
 * \author    Ivan Koryakovskiy <i.koryakovskiy@gmail.com>
 * \date      2016-02-09
 *
 * \copyright \verbatim
 * Copyright (c) 2016, Ivan Koryakovskiy
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

#include <grl/environments/communicator_env.h>
#include <iomanip>
#include <unistd.h>

using namespace grl;

REGISTER_CONFIGURABLE(CommunicatorEnvironment)

void CommunicatorEnvironment::request(ConfigurationRequest *config)
{
  config->push_back(CRP("converter", "converter", "Convert states and actions if needed", converter_, true));
  config->push_back(CRP("communicator", "communicator", "Comunicator which exchanges messages with an actual/virtual environment", communicator_));
  config->push_back(CRP("target_obs_dims", "Observation dimension of a target", target_obs_dims_, CRP::System));
  config->push_back(CRP("target_action_dims", "Action dimension of a target", target_action_dims_, CRP::System));
  config->push_back(CRP("benchmark_delays", "Observation dimension of a target", benchmark_delays_, CRP::System));
}

void CommunicatorEnvironment::configure(Configuration &config)
{
  converter_ = (StateActionConverter*)config["converter"].ptr();
  communicator_ = (Communicator*)config["communicator"].ptr();
  target_obs_dims_ = config["target_obs_dims"];
  target_action_dims_ = config["target_action_dims"];
  benchmark_delays_ = config["benchmark_delays"];

  if (converter_)
  {
    if (converter_->get_state_in_size() != target_obs_dims_)
      throw bad_param("environment/communicator:target_obs_dims");

    if (converter_->get_action_out_size() != target_action_dims_)
      throw bad_param("environment/communicator:target_action_dims_");
  }

  obs_conv_.resize(target_obs_dims_);
  action_conv_.resize(target_action_dims_);
  computation_stat_.setBufferLength(500);
}

void CommunicatorEnvironment::reconfigure(const Configuration &config)
{
}

void CommunicatorEnvironment::start(int test, Observation *obs)
{
  communicator_->recv(&obs_conv_);
  clock_gettime(CLOCK_MONOTONIC, &computation_begin_);
  if (converter_)
    converter_->convert_state(obs_conv_, obs->v);
  else
    *obs = obs_conv_;
  obs->absorbing = false;
}

double CommunicatorEnvironment::step(const Action &action, Observation *obs, double *reward, int *terminal)
{
  if (converter_)
    converter_->convert_action(action, action_conv_);
  else
    action_conv_ = action;

  if (benchmark_delays_)
  {
    timespec computation_end;
    clock_gettime(CLOCK_MONOTONIC, &computation_end);
    double computation_delay = (computation_end.tv_sec - computation_begin_.tv_sec)*1.0e6 + (static_cast<double>(computation_end.tv_nsec - computation_begin_.tv_nsec))/1.0e3;
    computation_stat_.addValue(computation_delay);
    std::cout << "Computation delay: " << computation_stat_.toStr("us") << std::endl;
  }

  communicator_->send(action_conv_);
  communicator_->recv(&obs_conv_);

  timespec computation_begin_prev = computation_begin_;
  clock_gettime(CLOCK_MONOTONIC, &computation_begin_);

  if (converter_)
    converter_->convert_state(obs_conv_, obs->v);
  else
    *obs = obs_conv_;
  obs->absorbing = false;

  double tau = (computation_begin_.tv_sec - computation_begin_prev.tv_sec) + (static_cast<double>(computation_begin_.tv_nsec - computation_begin_prev.tv_nsec))/1.0e9;
  //std::cout << "stg time: " << tau << std::endl;
  return tau;
}

