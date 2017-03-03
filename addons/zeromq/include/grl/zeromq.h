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

#include <grl/communicator.h>
#include <zmq_messenger.h>

#include <grl/representation.h>
#include <mutex>

namespace grl
{

// ZeroMQ generic communication class
class ZeromqCommunicator: public Communicator
{
  public:
    ZeromqCommunicator() : sync_(""), role_(0) { }

    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);

    // From Communicator
    virtual void send(const Vector &v) const;
    virtual bool recv(Vector *v) const;

  protected:
    ZeromqMessenger zmq_messenger_;
    std::string sync_;
    int role_;
};

// ZeroMQ publisher-subscriber communication class
class ZeromqPubSubCommunicator: public ZeromqCommunicator
{
  public:
    TYPEINFO("communicator/zeromq/pub_sub", "Zeromq class to establish a link by sending messages asynchronously (publisher/subscriber)")
    ZeromqPubSubCommunicator() : pub_("tcp://*:5561"), sub_("tcp://192.168.1.10:5562") {}

    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);

  protected:
    std::string pub_, sub_;
};

// ZeroMQ request-reply communication class
class ZeromqRequestReplyCommunicator: public ZeromqCommunicator
{
  public:
    TYPEINFO("communicator/zeromq/request_reply", "Zeromq class to establish a link by sending messages synchronously (request/reply)")
    ZeromqRequestReplyCommunicator() : addr_("tcp://localhost:5555") {}

    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);

  protected:
    std::string addr_;
};

}

#endif /* GRL_ZEROMQ_H_ */
