/** \file communivator.h
 * \brief Communicator environment header file.
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

#ifndef GRL_COMMUNICATOR_ENVIRONMENT_H_
#define GRL_COMMUNICATOR_ENVIRONMENT_H_

#include <grl/environment.h>
#include <grl/communicator.h>
#include <grl/converter.h>
#include <grl/statistics.h>
#include <time.h>

namespace grl
{

/// An environment which bridges actual environment with a middle layer environment by converting states and actions, and then sending and receiving messages
class CommunicatorEnvironment: public Environment
{
  public:
    TYPEINFO("environment/communicator", "Communicator environment which interects with a real environment by sending and receiving messages")
    CommunicatorEnvironment(): converter_(NULL), communicator_(NULL), target_obs_dims_(0), target_action_dims_(0), benchmark_delays_(0) {}

    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Environment
    virtual void start(int test, Observation *obs);
    virtual double step(const Action &action, Observation *obs, double *reward, int *terminal);

  protected:
    Vector obs_conv_, action_conv_;
    StateActionConverter *converter_;
    Communicator *communicator_;
    timespec computation_begin_;
    int target_obs_dims_, target_action_dims_;

    // benchmark communicaton delays
    int benchmark_delays_;
    CSimpleStat computation_stat_;
};

}

#endif /* GRL_COMMUNICATOR_ENVIRONMENT_H_ */
