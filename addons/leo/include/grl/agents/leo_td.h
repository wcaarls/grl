/** \file leo_td.h
 * \brief Leo temporal difference agent header file.
 *
 * \author    Ivan Koryakovskiy <i.koryakovskiy@tudelft.nl>
 * \date      2016-01-01
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

#ifndef GRL_LEO_TD_AGENT_H_
#define GRL_LEO_TD_AGENT_H_

#include <grl/agents/td.h>
#include <grl/signal.h>

namespace grl
{

/// Temporal difference learning agent for leo.
class LeoTDAgent : public TDAgent
{
  public:
    TYPEINFO("agent/leo/td", "Leo agent that learns from observed state transitions")

  protected: 
    VectorSignal *pub_transition_type_;

  public:
    LeoTDAgent() : pub_transition_type_(NULL) { }

    // From Configurable    
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Agent
    virtual void start(const Observation &obs, Action *action);
    virtual void step(double tau, const Observation &obs, double reward, Action *action);
};

}

#endif /* GRL_LEO_TD_AGENT_H_ */
