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

#include <zeromq.h>

using namespace grl;

REGISTER_CONFIGURABLE(ZeroMQPolicy)

void ZeroMQPolicy::request(ConfigurationRequest *config)
{
  config->push_back(CRP("observation_dims", "int.observation_dims", "Number of observation dimensions", CRP::System));
  config->push_back(CRP("action_dims", "int.action_dims", "Number of action dimensions", CRP::System));
}

void ZeroMQPolicy::configure(Configuration &config)
{
  // Read configuration
  action_dims_ = config["action_dims"];
  observation_dims_ = config["observation_dims"];

  //  Prepare our context
  context_ = new zmq::context_t(1);

  //prepare publisher
  publisher_ = new zmq::socket_t(*context_, ZMQ_PUB);
  publisher_->connect("tcp://localhost:5556");

  //prepare subscriber
  int confl = 1;
  subscriber_ = new zmq::socket_t(*this->context_, ZMQ_SUB);
  subscriber_->setsockopt(ZMQ_CONFLATE,&confl,sizeof(confl));// only receive last message
  subscriber_->connect("tcp://localhost:5555");
  subscriber_->setsockopt(ZMQ_SUBSCRIBE, "", 0);

  messageCount_ = 0;
}

void ZeroMQPolicy::reconfigure(const Configuration &config)
{
}

ZeroMQPolicy *ZeroMQPolicy::clone() const
{
  return new ZeroMQPolicy(*this);
}

void ZeroMQPolicy::act(double time, const Vector &in, Vector *out)
{
  messageCount_ += 1;

  out->resize(observation_dims_);
  zeromqMessages(in, out);

  if (messageCount_ - lastAction_ > 2000) // no message for 2 seconds
  {
    //reset action
    for (int i=0; i < action_dims_; i++)
      (*out)[i] = 0;
  }
}

// helper function to send a message using zeroMQ
void ZeroMQPolicy::send(DRL_MESSAGES::drl_unimessage &drlSendMessage)
{
  drlSendMessage.set_time_index(globalTimeIndex_);
  drlSendMessage.set_name("armstate");
  std::string msg_str;
  drlSendMessage.SerializeToString(&msg_str);
  zmq::message_t message (msg_str.size());
  memcpy ((void *) message.data (), msg_str.c_str(), msg_str.size());
  publisher_->send(message);
}

// helper function to receive a message using zeroMQ
bool ZeroMQPolicy::receive(DRL_MESSAGES::drl_unimessage* drlRecMessage)
{
  zmq::message_t update;
  bool received = subscriber_->recv(&update, ZMQ_DONTWAIT);
  if(received)
    drlRecMessage->ParseFromString(std::string(static_cast<char*>(update.data()), update.size()));
  return received;
}

// Helper function which deals with all communication
void ZeroMQPolicy::zeromqMessages(const Vector &in, Vector *out)
{
  DRL_MESSAGES::drl_unimessage drlRecMessage;
  bool received = receive(&drlRecMessage);

  if(received == true)
  {
    globalTimeIndex_ = drlRecMessage.time_index();
    if(drlRecMessage.type() == DRL_MESSAGES::drl_unimessage::MESSTR)
    {
      if (std::string(drlRecMessage.msgstr()).compare(std::string("senddim"))==0)
      {
        isConnected_ = false;
        //Prepare dimension message
        DRL_MESSAGES::drl_unimessage dimMessage;
        dimMessage.set_type(DRL_MESSAGES::drl_unimessage::DIMENSION);
        DRL_MESSAGES::drl_unimessage::Dimension* dimension = dimMessage.mutable_dimension();

        DRL_MESSAGES::drl_unimessage::Dimension::Component* compstate;
        compstate = dimension->add_component();
        compstate->set_component_name("state");
        compstate->add_component_dimension(observation_dims_);

        //DRL_MESSAGES::drl_unimessage::Dimension::Component* compaction;
        compstate = dimension->add_component();
        compstate->set_component_name("action");
        compstate->add_component_dimension(action_dims_);

        //send it
        send(dimMessage);
      }
      else if (std::string(drlRecMessage.msgstr()).compare(std::string("synched"))==0)
      {
        isConnected_ = true;
      }
    }
    else if(isConnected_ && drlRecMessage.type() == DRL_MESSAGES::drl_unimessage::CONTROLACTION)
    {
      lastAction_ = messageCount_;
      //Handle action message
      for (int i=0; i < std::min(action_dims_,  drlRecMessage.action().actions_size()); i++)
        (*out)[i] = SCALE[i] * std::max((float)-1.0, std::min(drlRecMessage.action().actions(i), (float)1.0)); // ???
    }
  }

  //Send state on every physics update when in synch
  if (isConnected_ == true)
  {
    DRL_MESSAGES::drl_unimessage stateMessage;
    stateMessage.set_type(DRL_MESSAGES::drl_unimessage::STATEPART);
    DRL_MESSAGES::drl_unimessage::GeneralStatePart* state = stateMessage.mutable_statepart();

    for (int i=0; i < observation_dims_/2; i++)
    {
      state->add_state(in[i]);
      state->add_first_derivative(in[i + observation_dims_/2]);
    }
    send(stateMessage);
  }
}
