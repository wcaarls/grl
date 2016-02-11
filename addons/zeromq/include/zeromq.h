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

#ifndef GRL_ZEROMQ_POLICY_H_
#define GRL_ZEROMQ_POLICY_H_

#include <grl/policy.h>
#include <zmq.hpp>
#include <drl_messages.pb.h>

namespace grl
{

/// ZeroMQ policy
class ZeroMQPolicy : public Policy
{
  public:
    TYPEINFO("policy/zeromq", "Fixed policy which sends and receives messages using ZeroMQ and protobuffers")

  protected:
    int action_dims_, observation_dims_;

    zmq::context_t* context_;
    zmq::socket_t* publisher_;
    zmq::socket_t* subscriber_;

    int lastAction_;
    int messageCount_;
    int globalTimeIndex_; // is it used??
    bool isConnected_;

    const double SCALE[4] = {0.6, 0.3, 0.15, 0.1}; // what is it for?

  public:
    ZeroMQPolicy() : observation_dims_(1), action_dims_(1) { }
  
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Policy
    virtual ZeroMQPolicy *clone() const;
    virtual void act(double time, const Vector &in, Vector *out);

  protected:
    void zeromqMessages(const Vector &in, Vector *out);
    bool receive(DRL_MESSAGES::drl_unimessage* drlRecMessage);
    void send(DRL_MESSAGES::drl_unimessage &drlSendMessage);
};

}

#endif /* GRL_ZEROMQ_POLICY_H_ */
