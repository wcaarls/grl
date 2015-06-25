/** \file agent.h
 * \brief Generic agent definition.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-01-22
 *
 * \copyright \verbatim
 * Copyright (c) 2015, Wouter Caarls
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
 
#ifndef GRL_AGENT_H_
#define GRL_AGENT_H_

#include <grl/configurable.h>

namespace grl
{

/// Interacts with an Environment.
class Agent : public Configurable
{
  public:
    virtual ~Agent() { }
    virtual Agent *clone() const = 0;
    
    /// Start the agent, returning the action for the first observation in an episode.
    virtual void start(const Vector &obs, Vector *action) = 0;
    
    /// Supply next state and reward, returning the next action.
    virtual void step(const Vector &obs, double reward, Vector *action) = 0;
    
    /// Signal an absorbing state.
    virtual void end(double reward) = 0;
    
    /// Progress report.
    virtual void report(std::ostream &os) const { }
};

/// Agent that is aware of its validity and can be used in conjunction with other agents.
class SubAgent : public Agent
{
  // TODO: SUB AGENTS SHOULD ALLOW FOR ACTIONS THAT ARE DIFFERENT FROM CHOSEN ACTION.
  public:
    virtual ~SubAgent() { }
    virtual SubAgent *clone() const = 0;
    
    /// Returns the agent's confidence for a certain observation.
    virtual double confidence(const Vector &obs) const = 0;
};

}

#endif /* GRL_AGENT_H_ */
