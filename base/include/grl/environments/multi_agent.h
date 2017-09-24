/** \file multi_agent.h
 * \brief Multi-agent environment header file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2017-09-15
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
 
#ifndef GRL_MULTI_AGENT_ENVIRONMENT_H_
#define GRL_MULTI_AGENT_ENVIRONMENT_H_

#include <itc/itc.h>
#include <grl/environment.h>

namespace grl
{

/// Environment that interacts with multiple agents at once.
/// When called using the regular interface, the agents are separated by their
/// thread ids.
class MultiAgentEnvironment : public Environment
{
  protected:
    Mutex mutex_;
    Instance<int> agent_id_;
    size_t agents_;

  public:
    MultiAgentEnvironment() : agents_(0) { }
  
    // From Environment
    virtual void start(int test, Observation *obs);
    virtual double step(const Action &action, Observation *obs, double *reward, int *terminal);
    
    /// Called to start the environment for specified agent.
    virtual void start(int agent_id, int test, Observation *obs) = 0;
  
    /// Called to step the environment for specified agent.
    virtual double step(int agent_id, const Action &action, Observation *obs, double *reward, int *terminal) = 0;
};

/// MultiAgentEnvironment that runs the simulation in a separate thread which interacts with
/// the experiment through startAgent(), stepAgent() and endAgent(). The episode ends when
/// run() exits. Before that, all agents must be terminated.
class ThreadedMultiAgentEnvironment : public MultiAgentEnvironment, public itc::Thread
{
  protected:
    enum agentState {asReady, asRunning, asTerminated, asStopped};
  
    struct ObservationRewardTerminal
    {
      Observation obs;
      double reward;
      int terminal;
      
      ObservationRewardTerminal(const Observation &_obs=Observation(), double _reward=0., int _terminal=0) :
        obs(_obs), reward(_reward), terminal(_terminal) { }
    };
    
    std::vector<agentState> state_;
    std::vector<itc::SharedVariable<ObservationRewardTerminal> > observation_;
    std::vector<itc::SharedVariable<Action> > action_;
    
  protected:
    // From MultiAgentEnvironment
    virtual void start(int agent_id, int test, Observation *obs);
    virtual double step(int agent_id, const Action &action, Observation *obs, double *reward, int *terminal);
  
    /// Called by run() to get action from specified agent.
    virtual void getAgentAction(int agent_id, Observation &obs, double reward, int terminal, Action *action);

    /// Prepare agents. Must be called during construction and by run() at the start of an episode.
    virtual void prepareAgents(size_t agents);

    /// Stop running agents. Must be called by run() at the end of an episode.
    virtual void stopAgents();
  protected:
    /// Override to implement simulation. In this function, call getAgentAction() to interact with the agents.
    /// At the start of an episode, call prepareAgents() and at the end call stopAgents().
    virtual void run() = 0;
};

}

#endif /* GRL_MULTI_AGENT_ENVIRONMENT_H_ */
