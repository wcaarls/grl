/** \file zeromq.cpp
 * \brief ZeroMQ policy source file.
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

#include <grl/zeromq.h>

using namespace grl;

REGISTER_CONFIGURABLE(ZeromqPubSubCommunicator)
REGISTER_CONFIGURABLE(ZeromqRequestReplyCommunicator)

void ZeromqCommunicator::request(ConfigurationRequest *config)
{
  config->push_back(CRP("role", "Role of the zeromq (Pub/Sub, Request/Reply)", "", CRP::Configuration, {"NONE", "ZMQ_SUB", "ZMQ_PUB","ZMQ_REQ","ZMQ_REP"}));
  config->push_back(CRP("sync", "Syncronization ip address", sync_));
}

void ZeromqCommunicator::configure(Configuration &config)
{
  std::string role = config["role"].str();
  sync_ = config["sync"].str();

  if (role == "ZMQ_SUB")
    role_ = ZMQ_SUB;
  if (role == "ZMQ_PUB")
    role_ = ZMQ_PUB;
  if (role == "ZMQ_REQ")
    role_ = ZMQ_REQ;
  if (role == "ZMQ_REP")
    role_ = ZMQ_REP;
}

void ZeromqCommunicator::send(const Vector v) const
{
  zmq_messenger_.send(reinterpret_cast<const void*>(v.data()), v.cols()*sizeof(double));
}

bool ZeromqCommunicator::recv(Vector *v) const
{
  Vector v_rc;
  v_rc.resize(v->size());
  CRAWL(v_rc.cols());
  bool rc = zmq_messenger_.recv(reinterpret_cast<void*>(v_rc.data()), v_rc.cols()*sizeof(double), 0);
  if (rc)
    *v = v_rc;
  return rc;
}

/////////////////////////////////////////////////////////

void ZeromqPubSubCommunicator::request(ConfigurationRequest *config)
{
  ZeromqCommunicator::request(config);
  config->push_back(CRP("pub", "Publisher address", pub_));
  config->push_back(CRP("sub", "subscriber address", sub_));
}

void ZeromqPubSubCommunicator::configure(Configuration &config)
{
  ZeromqCommunicator::configure(config);

  // possible Leo addresses
  // zmq_.init("tcp://*:5561", "tcp://192.168.2.210:5562", "tcp://192.168.2.210:5560", ZMQ_SYNC_SUB); // wifi
  // zmq_.init("tcp://*:5561",  "tcp://192.168.1.10:5562",  "tcp://192.168.1.10:5560", ZMQ_SYNC_SUB); // ethernet

  pub_ = config["pub"].str();
  sub_ = config["sub"].str();

  // initialize zmq
  zmq_messenger_.start(role_, pub_.c_str(), sub_.c_str(), sync_.c_str());
}

////////////////////////////////////////////////////////

void ZeromqRequestReplyCommunicator::request(ConfigurationRequest *config)
{
  ZeromqCommunicator::request(config);
  config->push_back(CRP("addr", "Address", addr_));
}

void ZeromqRequestReplyCommunicator::configure(Configuration &config)
{
  ZeromqCommunicator::configure(config);

  addr_ = config["addr"].str();

  // initialize zmq
  zmq_messenger_.start(role_, addr_.c_str(), NULL, sync_.c_str());
}


