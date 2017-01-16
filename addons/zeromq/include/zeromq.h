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
#include <grl/converter.h>
#include <drl_messages.pb.h>
#include <time.h>

namespace grl
{

/// Base communicator class
class Communicator: public Configurable
{
public:
  virtual ~Communicator() { }

  /// Send data.
  virtual void send(const Vector v) const = 0;

  /// Receive data.
  virtual bool recv(Vector &v) const = 0;
};

// ZeroMQ generic communication class
class ZeromqCommunicator: public Communicator
{
  public:
    ZeromqCommunicator() : sync_("") { }

    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);

    // From Communicator
    virtual void send(const Vector v) const;
    virtual bool recv(Vector &v) const;

  protected:
    ZeromqMessenger zmq_messenger_;
    std::string sync_;
    int pattern_;
};

// ZeroMQ publisher-subscriber communication class
class ZeromqPubSubCommunicator: public ZeromqCommunicator
{
  public:
    TYPEINFO("communicator/zeromq/pub_sub", "A zeromq class capable to establish a link by events and send messages asynchronously (publisher/subscriber)")
    ZeromqPubSubCommunicator() : pub_("tcp://*:5561"), sub_("tcp://192.168.1.10:5562") {}

    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);

  protected:
    std::string pub_, sub_;
};
class ZeromqRequestReplyCommunicator: public ZeromqCommunicator
{
  public:
    TYPEINFO("communicator/zeromq/request_reply", "A zeromq class capable to establish a link by events and send messages asynchronously (request/reply)")
    ZeromqRequestReplyCommunicator() : cli_("tcp://localhost:5555") {}

    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);

  protected:
    std::string cli_;
};
// @Divyam, derive your communicator class from ZeromqCommunicator

/// An environment which bridges actual environment with a middle layer environment by converting states and actions, and then sending and receiving messages
class CommunicatorEnvironment: public Environment
{
  public:
    TYPEINFO("environment/communicator", "Communicator environment which interects with a real environment by sending and receiving messages")
    CommunicatorEnvironment(): target_obs_dims_(0), target_action_dims_(0) {}

    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Environment
    virtual void start(int test, Vector *obs);
    virtual double step(const Vector &action, Vector *obs, double *reward, int *terminal);

  protected:
    Vector obs_conv_, action_conv_;
    StateActionConverter *converter_;
    Communicator *communicator_;
    timespec time_begin_;
    int target_obs_dims_, target_action_dims_;
};

/// ZeroMQ agent
class ZeromqAgent : public Agent
{
  public:
    TYPEINFO("agent/zeromq", "Zeromq Agent which interects with a python by sending and receiving messages")
    ZeromqAgent() : observation_dims_(1), action_dims_(1) { }

    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Policy
    virtual TransitionType start(const Vector &obs, Vector *action);
    virtual TransitionType step(double tau, const Vector &obs, double reward, Vector *action);
    virtual void end(double tau, const Vector &obs, double reward);


  protected:
    int action_dims_, observation_dims_;
    Vector action_min_, action_max_;
    Communicator *communicator_;


};

}

#endif /* GRL_ZEROMQ_H_ */
