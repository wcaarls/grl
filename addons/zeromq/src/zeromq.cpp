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
#include <iomanip>

using namespace grl;

REGISTER_CONFIGURABLE(ZeromqCommunicator)
REGISTER_CONFIGURABLE(CommunicatorEnvironment)

void ZeromqCommunicator::request(ConfigurationRequest *config)
{
  config->push_back(CRP("pub", "Publisher address", pub_));
  config->push_back(CRP("sub", "subscriber address", sub_));
  config->push_back(CRP("event", "Event address", event_));
  config->push_back(CRP("event_mode", "Event mode", event_mode_));
}

void ZeromqCommunicator::configure(Configuration &config)
{
  //  zmq_.init("tcp://*:5561", "tcp://192.168.2.210:5562", "tcp://192.168.2.210:5560", ZMQ_SYNC_SUB); // wifi
  //zmq_.init("tcp://*:5561",  "tcp://192.168.1.10:5562",  "tcp://192.168.1.10:5560", ZMQ_SYNC_SUB); // ethernet

  pub_ = config["pub"].str();
  sub_ = config["sub"].str();
  event_ = config["event"].str();
  event_mode_ = config["event_mode"].str();

  // find event mode
  int mode = 0;
  if (event_mode_ == "ZMQ_SYNC_SUB")
    mode |= ZMQ_SYNC_SUB;
  if (event_mode_ == "ZMQ_SYNC_PUB")
    mode |= ZMQ_SYNC_PUB;

  // initialize zmq
  zmq_messenger_.init(pub_.c_str(), sub_.c_str(), event_.c_str(), mode);
}

void ZeromqCommunicator::reconfigure(const Configuration &config)
{
}

ZeromqCommunicator *ZeromqCommunicator::clone() const
{
  ZeromqCommunicator* me = new ZeromqCommunicator();
  return me;
}

void ZeromqCommunicator::send(const Vector v) const
{
  zmq_messenger_.send(reinterpret_cast<const void*>(v.data()), v.cols()*sizeof(double));
}

bool ZeromqCommunicator::recv(Vector &v) const
{
  bool rc = zmq_messenger_.recv(reinterpret_cast<void*>(v.data()), v.cols()*sizeof(double)); // ZMQ_NOBLOCK
  //std::cout << std::fixed << std::setprecision(2) << std::right << std::setw(7) << v << std::endl;
  return rc;
}

//////////////////////////////////////////////////////////
void CommunicatorEnvironment::request(ConfigurationRequest *config)
{
  config->push_back(CRP("communicator", "communicator", "Comunicator which exchanges messages with an actual environment", communicator_));
}

void CommunicatorEnvironment::configure(Configuration &config)
{
  communicator_ = (Communicator*)config["communicator"].ptr();
}

void CommunicatorEnvironment::reconfigure(const Configuration &config)
{
}

CommunicatorEnvironment *CommunicatorEnvironment::clone() const
{
  CommunicatorEnvironment *me = new CommunicatorEnvironment();
  return me;
}

void CommunicatorEnvironment::start(int test, Vector *obs)
{
  communicator_->recv(*obs);
}

double CommunicatorEnvironment::step(const Vector &action, Vector *obs, double *reward, int *terminal)
{
  communicator_->send(action);
  communicator_->recv(*obs);
}

//////////////////////////////////////////////////////////
void ZeromqAgent::request(ConfigurationRequest *config)
{
  config->push_back(CRP("observation_dims", "int.observation_dims", "Number of observation dimensions", observation_dims_, CRP::System));
  config->push_back(CRP("action_dims", "int.action_dims", "Number of action dimensions", action_dims_, CRP::System));
  config->push_back(CRP("action_min", "vector.action_min", "Lower limit of action", action_min_, CRP::System));
  config->push_back(CRP("action_max", "vector.action_max", "Upper limit of action", action_max_, CRP::System));
}

void ZeromqAgent::configure(Configuration &config)
{
  // Read configuration
  action_dims_ = config["action_dims"];
  observation_dims_ = config["observation_dims"];
  action_min_ = config["action_min"].v();
  action_max_ = config["action_max"].v();

  //  Prepare our context
  context_ = new zmq::context_t(1);

  //prepare publisher
  publisher_ = new zmq::socket_t(*context_, ZMQ_PUB);
  publisher_->connect("tcp://localhost:5556");

  //prepare subscriber
  int confl = 1;
  subscriber_ = new zmq::socket_t(*this->context_, ZMQ_SUB);
  //subscriber_->setsockopt(ZMQ_CONFLATE,&confl,sizeof(confl));// only receive last message
  subscriber_->connect("tcp://localhost:5555");
  subscriber_->setsockopt(ZMQ_SUBSCRIBE, "", 0);

  // Establish connection
  init();
  sleep(1);
}

void ZeromqAgent::reconfigure(const Configuration &config)
{
}

ZeromqAgent *ZeromqAgent::clone() const
{
  return new ZeromqAgent(*this);
}

void ZeromqAgent::start(const Vector &obs, Vector *action)
{
  action->resize(action_dims_);
  communicate(obs, 0, 0, action);
}

void ZeromqAgent::step(double tau, const Vector &obs, double reward, Vector *action)
{
  action->resize(action_dims_);
  communicate(obs, reward, 0, action);
}

void ZeromqAgent::end(double tau, const Vector &obs, double reward)
{
  communicate(obs, reward, 1, NULL);
}

// helper function to send a message using zeroMQ
void ZeromqAgent::send(DRL_MESSAGES::drl_unimessage &drlSendMessage)
{
  TRACE("Send time index: " << globalTimeIndex_);
  drlSendMessage.set_time_index(globalTimeIndex_);
  drlSendMessage.set_name("state");
  std::string msg_str;
  drlSendMessage.SerializeToString(&msg_str);
  zmq::message_t message (msg_str.size());
  memcpy ((void *) message.data (), msg_str.c_str(), msg_str.size());
  publisher_->send(message);
}

// helper function to receive a message using zeroMQ
bool ZeromqAgent::receive(DRL_MESSAGES::drl_unimessage* drlRecMessage)
{
  zmq::message_t update;
  bool received = subscriber_->recv(&update, ZMQ_DONTWAIT);
  //bool received = subscriber_->recv(&update);
  if(received)
    drlRecMessage->ParseFromString(std::string(static_cast<char*>(update.data()), update.size()));
  return received;
}

void ZeromqAgent::receive(const DRL_MESSAGES::drl_unimessage_Type type,
                           const char *msgstr,
                           DRL_MESSAGES::drl_unimessage &msg)
{
  while (1)
  {
    if (receive(&msg))
    {
      globalTimeIndex_ = msg.time_index();
      TRACE("Recived msg type = "<< msg.type() << "; msgstr = " << msg.msgstr());
      TRACE("Recieved time index: " << globalTimeIndex_);
      switch (msg.type())
      {
        case DRL_MESSAGES::drl_unimessage::MESSTR:
          if (std::string(msg.msgstr()).compare(std::string("senddim"))==0)
          {
            if (globalTimeIndex_ < 1)
            {
              // Prepare and send a dimension message
              DRL_MESSAGES::drl_unimessage dimMessage;
              dimMessage.set_type(DRL_MESSAGES::drl_unimessage::DIMENSION);
              DRL_MESSAGES::drl_unimessage::Dimension* dimension = dimMessage.mutable_dimension();

              DRL_MESSAGES::drl_unimessage::Dimension::Component* compstate;
              compstate = dimension->add_component();
              compstate->set_component_name("state");
              compstate->add_component_dimension(observation_dims_);

              compstate = dimension->add_component();
              compstate->set_component_name("action");
              compstate->add_component_dimension(action_dims_);

              send(dimMessage);
              TRACE("Dimentions were sent");
            }
          }
          else if (std::string(msg.msgstr()).compare(std::string("synched"))==0)
          {
            if (globalTimeIndex_ < 1)
            {
              globalTimeIndex_ = 1;
              isConnected_ = true;
              TRACE("synched received");
            }
          }
          break;

        case DRL_MESSAGES::drl_unimessage::CONTROLACTION:
          TRACE("action received");
          break;
      }

      // complete reception if message we were waiting has arrived
      if(msg.type() == type)
      {
        if ((msgstr == NULL) || (msgstr == '\0') || (std::string(msg.msgstr()).compare(std::string(msgstr))==0))
          return;
      }
    }
  }
}

void ZeromqAgent::init()
{
  isConnected_ = false;
  DRL_MESSAGES::drl_unimessage msg;
  receive(DRL_MESSAGES::drl_unimessage::MESSTR, "senddim", msg);
  receive(DRL_MESSAGES::drl_unimessage::MESSTR, "synched", msg);
}

// Helper function which deals with all communication
void ZeromqAgent::communicate(const Vector &in, double reward, double terminal, Vector *out)
{
  if (!isConnected_)
    return;

  // Send state on every physics update when in synch
  DRL_MESSAGES::drl_unimessage stateMessage;
  stateMessage.set_type(DRL_MESSAGES::drl_unimessage::STATEPART);
  DRL_MESSAGES::drl_unimessage::GeneralStatePart* state = stateMessage.mutable_statepart();
  for (int i = 0; i < observation_dims_; i += 2)
  {
    state->add_state(in[i]);
    state->add_first_derivative(in[i+1]);
  }
  send(stateMessage);
  TRACE("State was sent");

  // Send reward and terminal
  DRL_MESSAGES::drl_unimessage rwtMessage;
  rwtMessage.set_type(DRL_MESSAGES::drl_unimessage::REWARDTERMINAL);
  DRL_MESSAGES::drl_unimessage::RewardTerminal* rwt = rwtMessage.mutable_rwt();
  rwt->set_reward(reward);
  rwt->set_terminal(terminal);
  send(rwtMessage);
  TRACE("Reward and terminal were sent");

  if (out)
  {
    // Receive action back
    DRL_MESSAGES::drl_unimessage msg;
    receive(DRL_MESSAGES::drl_unimessage::CONTROLACTION, NULL, msg);
    //Handle action message
    for (int i = 0; i < std::min(action_dims_, msg.action().actions_size()); i++)
    {
      double a = std::max(static_cast<float>(-1.0), std::min(msg.action().actions(i), static_cast<float>(1.0)));
      (*out)[i] = (action_max_[i] - action_min_[i])*(a+1.0)/2.0 + action_min_[i];
      TRACE("Action: " << a);
    }
    TRACE("Action is received: " << *out);
  }
}
