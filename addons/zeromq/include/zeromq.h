/** \file zeromq.h
 * \brief ZeroMQ policy header file.
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

#ifndef GRL_ZEROMQ_H_
#define GRL_ZEROMQ_H_

#include <zmq_messenger.h>
#include <grl/environment.h>
#include <grl/agent.h>
#include <drl_messages.pb.h>

namespace grl
{

/// Base communicator class
class Communicator: public Configurable
{
public:
  virtual ~Communicator() { }
  virtual Communicator *clone() const = 0;

  /// Send data.
  virtual void send(const Vector v) const = 0;

  /// Receive data.
  virtual bool recv(Vector &v) const = 0;
};

// ZeroMQ configurable communication class
class ZeromqCommunicator: public Communicator
{
  public:
    TYPEINFO("communicator/zeromq", "A zeromq class capable to establish a link by events and send messages asynchronously (publisher/subscriber)")
    ZeromqCommunicator() : pub_("tcp://*:5561"), sub_("tcp://192.168.1.10:5562"), event_(""), event_mode_("") {}

    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Environment
    virtual ZeromqCommunicator *clone() const;

    virtual void send(const Vector v) const;
    virtual bool recv(Vector &v) const;

  protected:
    std::string pub_, sub_, event_, event_mode_;
    ZeromqMessenger zmq_messenger_;
};

/// An environment which bridges actual environment with an agent by sending and receiving messages
class CommunicatorEnvironment: public Environment
{
  public:
    TYPEINFO("environment/communicator", "Communicator environment which interects with a real environment by sending and receiving messages")
    CommunicatorEnvironment() {}

    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Environment
    virtual CommunicatorEnvironment *clone() const;

    virtual void start(int test, Vector *obs);
    virtual double step(const Vector &action, Vector *obs, double *reward, int *terminal);

  protected:
    Communicator *communicator_;
};

/// ZeroMQ agent
class ZeromqAgent : public Agent
{
  public:
    TYPEINFO("agent/zeromq", "Agent which sends and receives messages using ZeroMQ and protobuffers")

  protected:
    int action_dims_, observation_dims_;
    Vector action_min_, action_max_;

    zmq::context_t* context_;
    zmq::socket_t* publisher_;
    zmq::socket_t* subscriber_;

    int lastAction_;
    int globalTimeIndex_;
    bool isConnected_;

  public:
    ZeromqAgent() : observation_dims_(1), action_dims_(1), isConnected_(false), globalTimeIndex_(-1) { }
  
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Policy
    virtual ZeromqAgent *clone() const;
    virtual void start(const Vector &obs, Vector *action);
    virtual void step(double tau, const Vector &obs, double reward, Vector *action);
    virtual void end(double tau, const Vector &obs, double reward);

    //virtual void act(double time, const Vector &in, Vector *out);

  protected:
    void init();
    void communicate(const Vector &in, double reward, double terminal, Vector *out);
    bool receive(DRL_MESSAGES::drl_unimessage* drlRecMessage);
    void send(DRL_MESSAGES::drl_unimessage &drlSendMessage);

    void receive(const DRL_MESSAGES::drl_unimessage_Type type, const char *msgstr, DRL_MESSAGES::drl_unimessage &msg);
};

}

#endif /* GRL_ZEROMQ_H_ */
