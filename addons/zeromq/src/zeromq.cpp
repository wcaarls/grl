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
REGISTER_CONFIGURABLE(CommunicatorRepresentation)

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

void ZeromqCommunicator::send(const Vector &v) const
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

//////////////////////////////////////////////////////////

void CommunicatorRepresentation::request(const std::string &role, ConfigurationRequest *config)
{
  if (role == "action")
  {
    config->push_back(CRP("inputs", "int.observation_dims", "Number of input dimensions", (int)inputs_, CRP::System, 1));
    config->push_back(CRP("outputs", "int.action_dims", "Number of output dimensions", (int)outputs_, CRP::System, 1));
  }
  else
  {
    if (role == "transition" || role == "value/action")
      config->push_back(CRP("inputs", "int.observation_dims+int.action_dims", "Number of input dimensions", (int)inputs_, CRP::System, 1));
    else if (role == "value/state")
      config->push_back(CRP("inputs", "int.observation_dims", "Number of input dimensions", (int)inputs_, CRP::System, 1));
    else
      config->push_back(CRP("inputs", "Number of input dimensions", (int)inputs_, CRP::System, 1));
      
    if (role == "transition")
      config->push_back(CRP("outputs", "int.observation_dims+2", "Number of output dimensions", (int)outputs_, CRP::System, 1));
    else
      config->push_back(CRP("outputs", "Number of output dimensions", (int)outputs_, CRP::System, 1));
  }
  
  config->push_back(CRP("communicator", "communicator", "Communicator which exchanges messages with the out-of-process representation", communicator_));
}

void CommunicatorRepresentation::configure(Configuration &config)
{
  communicator_ = (Communicator*)config["communicator"].ptr();
  inputs_ = config["inputs"];
  outputs_ = config["outputs"];
  
  Configuration comcfg_in, comcfg_out;
  
  comcfg_in.set("inputs", inputs_);
  comcfg_in.set("outputs", outputs_);
  
  communicator_->setup(comcfg_in, &comcfg_out);
}

void CommunicatorRepresentation::reconfigure(const Configuration &config)
{
}

double CommunicatorRepresentation::read(const ProjectionPtr &projection, Vector *result, Vector *stddev) const
{
  std::lock_guard<std::mutex> lock(mutex_);

  IndexProjection *ip = dynamic_cast<IndexProjection*>(projection.get());
  if (ip)
  {
    if (ip->indices.size() != 1)
      throw Exception("Communicator representation does not support multi-index projections");
      
    //CRAWL("Reading " << ip->indices[0]);
    communicator_->send(VectorConstructor(mtRead, (double)ip->indices[0]));
  }
  else
  {
    VectorProjection *vp = dynamic_cast<VectorProjection*>(projection.get());
    if (vp)
    {
      if (vp->vector.size() != inputs_)
      {
        ERROR("Projection size " << vp->vector.size() << " doesn't match input size " << inputs_);
        throw bad_param("representation/communicator:inputs");
      }
    
      //CRAWL("Reading " << vp->vector);
      communicator_->send(extend(VectorConstructor(mtRead), vp->vector));
    }
    else
      throw Exception("Communicator representation only supports index or vector projections");
  }
  
  result->resize(outputs_);
  communicator_->recv(result);
  //CRAWL("Result " << *result);

  if (stddev)
    *stddev = Vector();
    
  return (*result)[0];
}

void CommunicatorRepresentation::write(const ProjectionPtr projection, const Vector &target, const Vector &alpha)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (target.size() != outputs_)
    throw bad_param("representation/communicator:outputs");

  if (target.size() != alpha.size())
    throw Exception("Learning rate vector does not match target vector");

  IndexProjection *ip = dynamic_cast<IndexProjection*>(projection.get());
  if (ip)
  {
    if (ip->indices.size() != 1)
      throw Exception("Communicator representation does not support multi-index projections");
      
    Vector v(2 + target.size() + alpha.size());
    v << mtWrite, ip->indices[0], target, alpha;

    //CRAWL("Writing " << ip->indices[0] << " -" << alpha << "> " << target);
    communicator_->send(v);
  }
  else
  {
    VectorProjection *vp = dynamic_cast<VectorProjection*>(projection.get());
    if (vp)
    {
      if (vp->vector.size() != inputs_)
      {
        ERROR("Projection size " << vp->vector.size() << " doesn't match input size " << inputs_);
        throw bad_param("representation/communicator:inputs");
      }
        
      Vector v(1 + vp->vector.size() + target.size() + alpha.size());
      v << mtWrite, vp->vector, target, alpha;
    
      //CRAWL("Writing " << vp->vector << " -" << alpha << "> " << target);
      communicator_->send(v);
    }
    else
      throw Exception("Communicator representation only supports index or vector projections");
  }

  Vector _;
  communicator_->recv(&_);
}

void CommunicatorRepresentation::update(const ProjectionPtr projection, const Vector &delta)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (delta.size() != outputs_)
    throw bad_param("representation/communicator:outputs");

  IndexProjection *ip = dynamic_cast<IndexProjection*>(projection.get());
  if (ip)
  {
    if (ip->indices.size() != 1)
      throw Exception("Communicator representation does not support multi-index projections");
      
    //CRAWL("Updating " << ip->indices[0] << " + " << delta);
    communicator_->send(extend(VectorConstructor(mtUpdate, (double)ip->indices[0]), delta));
  }
  else
  {
    VectorProjection *vp = dynamic_cast<VectorProjection*>(projection.get());
    if (vp)
    {
      if (vp->vector.size() != inputs_)
      {
        ERROR("Projection size " << vp->vector.size() << " doesn't match input size " << inputs_);
        throw bad_param("representation/communicator:inputs");
      }
        
      Vector v(1 + vp->vector.size() + delta.size());
      v << mtUpdate, vp->vector, delta;
    
      //CRAWL("Updating " << vp->vector << " + " << delta);
      communicator_->send(v);
    }
    else
      throw Exception("Communicator representation only supports index or vector projections");
  }

  Vector _;
  communicator_->recv(&_);
}

void CommunicatorRepresentation::finalize()
{
  std::lock_guard<std::mutex> lock(mutex_);

  Vector _;

  //CRAWL("Finalizing");
  communicator_->send(VectorConstructor(mtFinalize));
  communicator_->recv(&_);
}
