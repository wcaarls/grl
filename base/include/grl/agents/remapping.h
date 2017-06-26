/** \file remapping.h
 * \brief Remapping agent header file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2017-06-26
 *
 * \copyright \verbatim
 * Copyright (c) 2017, Wouter Caarls
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

#ifndef GRL_REMAPPING_AGENT_H_
#define GRL_REMAPPING_AGENT_H_

#include <grl/agent.h>
#include <grl/policy.h>
#include <grl/signal.h>

namespace grl
{

/// Agent that remaps actions.
class RemappingAgent : public Agent
{
  public:
    TYPEINFO("agent/remapping", "Agent that remaps actions")

  protected:
    Agent *agent_;
    Mapping *mapping_;
    
  public:
    RemappingAgent() : agent_(NULL), mapping_(NULL) { }
  
    // From Configurable    
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Agent
    virtual void start(const Observation &obs, Action *action);
    virtual void step(double tau, const Observation &obs, double reward, Action *action);
    virtual void end(double tau, const Observation &obs, double reward);
};

}

#endif /* GRL_REMAPPING_AGENT_H_ */
