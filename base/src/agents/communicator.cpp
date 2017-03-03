/** \file communicator.cpp
 * \brief Communicator agent source file.
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

#include <grl/agents/communicator.h>
#include <iomanip>
#include <unistd.h>

using namespace grl;

REGISTER_CONFIGURABLE(CommunicatorAgent)

void CommunicatorAgent::request(ConfigurationRequest *config)
{
  config->push_back(CRP("communicator", "communicator", "Comunicator which exchanges messages with an actual/virtual environment", communicator_));
  config->push_back(CRP("observation_dims", "int.observation_dims", "Number of observation dimensions", observation_dims_, CRP::System));
  config->push_back(CRP("action_dims", "int.action_dims", "Number of action dimensions", action_dims_, CRP::System));
  config->push_back(CRP("action_min", "vector.action_min", "Lower limit of action", action_min_, CRP::System));
  config->push_back(CRP("action_max", "vector.action_max", "Upper limit of action", action_max_, CRP::System));
  config->push_back(CRP("test", "int.test", "Selection of a learning/testing agent role", test_, CRP::System));
}

void CommunicatorAgent::configure(Configuration &config)
{
  // Read configuration
  action_dims_ = config["action_dims"];
  observation_dims_ = config["observation_dims"];
  action_min_ = config["action_min"].v();
  action_max_ = config["action_max"].v();
  communicator_ = (Communicator*)config["communicator"].ptr();
  test_ = config["test"];
}

void CommunicatorAgent::reconfigure(const Configuration &config)
{
}

void CommunicatorAgent::start(const Observation &obs, Action *action)
{
  action->v.resize(action_dims_);
  action->type = atUndefined;

  Vector v(obs.v.cols()+1);
  v << test_, obs.v;
  communicator_->send(v);
  communicator_->recv(&(action->v));
}

void CommunicatorAgent::step(double tau, const Observation &obs, double reward, Action *action)
{
  action->v.resize(action_dims_);
  action->type = atUndefined;
  
  Vector v(obs.v.cols()+3);
  v << test_, obs.v, reward, 0;
  communicator_->send(v);
  communicator_->recv(&(action->v));
}

void CommunicatorAgent::end(double tau, const Observation &obs, double reward)
{
  Vector temp;

  Vector v(obs.v.cols()+3);
  v << test_, obs.v, reward, 2;
  communicator_->send(v);
  communicator_->recv(&temp);
}


