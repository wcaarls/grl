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
    
    /// Start the agent, returning the action for the first observation in an episode.
    virtual void start(const Vector &obs, Vector *action) = 0;
    
    /**
     * \brief Supply next state and reward, returning the next action.
     *
     * \note action is an inout parameter, the input being a previous or suggested action.
     */
    virtual void step(double tau, const Vector &obs, double reward, Vector *action) = 0;
    
    /// Signal an absorbing state.
    virtual void end(double tau, const Vector &obs, double reward) = 0;
    
    /// Progress report.
    virtual void report(std::ostream &os) { }
};

/// Agent that is aware of its validity and can be used in conjunction with other agents.
class SubAgent : public Agent
{
  // TODO: SUB AGENTS SHOULD ALLOW FOR ACTIONS THAT ARE DIFFERENT FROM CHOSEN ACTION.
  public:
    virtual ~SubAgent() { }

    // From Agent
    virtual void start(const Vector &obs, Vector *action)
    {
      double confidence;
      start(obs, action, &confidence);
    }

    virtual void step(double tau, const Vector &obs, double reward, Vector *action)
    {
      double confidence;
      step(tau, obs, reward, action, &confidence);
    }

    /**
     * Start function that also returns confidence. 
     * Note that unlike the confidence() function, calling this function
     * executes a learning step, even when the returned confidence is 0.
     */
    virtual void start(const Vector &obs, Vector *action, double *conf)
    {
      start(obs, action);
      *conf = confidence(obs);
    }

    /**
     * Step function that also returns confidence. 
     * Note that unlike the confidence() function, calling this function
     * executes a learning step, even when the returned confidence is 0.
     */
    virtual void step(double tau, const Vector &obs, double reward, Vector *action, double *conf)
    {
      step(tau, obs, reward, action);
      *conf = confidence(obs);
    }

    /// Returns the agent's confidence for a certain observation.
    virtual double confidence(const Vector &obs) const = 0;
};

}

#endif /* GRL_AGENT_H_ */
