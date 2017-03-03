/** \file communicator.h
 * \brief Communicator agent header file.
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

#ifndef GRL_COMMUNICATOR_AGENT_H_
#define GRL_COMMUNICATOR_AGENT_H_

#include <grl/agent.h>
#include <grl/communicator.h>

namespace grl
{

/// An agent which connects GRL to a remote agent
class CommunicatorAgent : public Agent
{
  public:
    TYPEINFO("agent/communicator", "Communicator agent which connects GRL to a remote agent")
    CommunicatorAgent() : action_dims_(1), observation_dims_(1), test_(0) { }

    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Policy
    virtual void start(const Observation &obs, Action *action);
    virtual void step(double tau, const Observation &obs, double reward, Action *action);
    virtual void end(double tau, const Observation &obs, double reward);

  protected:
    int action_dims_, observation_dims_;
    Vector action_min_, action_max_;
    Communicator *communicator_;
    int test_;
};

}

#endif /* GRL_COMMUNICATOR_AGENT_H_ */
