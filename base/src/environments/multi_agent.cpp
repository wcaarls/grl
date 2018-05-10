/** \file multi_agent.cpp
 * \brief Multi-agent environment source file.
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

#include <unistd.h>

#include <grl/environments/multi_agent.h>

using namespace grl;

// MultiAgentEnvironment
void MultiAgentEnvironment::start(int test, Observation *obs)
{
  if (!*agent_id_)
  {
    Guard guard(mutex_);
    *agent_id_ = ++agents_;
  }

  start(*agent_id_-1, test, obs);
}

double MultiAgentEnvironment::step(const Action &action, Observation *obs, double *reward, int *terminal)
{
  return step(*agent_id_-1, action, obs, reward, terminal); 
}

// ThreadedMultiAgentEnvironment
void ThreadedMultiAgentEnvironment::start(int agent_id, int test, Observation *obs)
{
  grl_assert(*agent_id_ < state_.size());
  while (state_[*agent_id_] != asReady) usleep(10);
  
  state_[*agent_id_] = asRunning;

  ObservationRewardTerminal msg = observation_[*agent_id_].read();
  *obs = msg.obs;
  
  if (msg.terminal)
    state_[*agent_id_] = asTerminated;
}

double ThreadedMultiAgentEnvironment::step(int agent_id, const Action &action, Observation *obs, double *reward, int *terminal)
{
  grl_assert(*agent_id_ < state_.size());
  
  if (state_[*agent_id_] == asRunning)
  {
    action_[*agent_id_].write(action);
    ObservationRewardTerminal msg = observation_[*agent_id_].read();

    *obs = msg.obs;
    *reward = msg.reward;
    *terminal = msg.terminal;
  }
  else
  {
    *obs = Observation();
    *reward = 0.;
    *terminal = 2;
  }

  if (*terminal)
    state_[*agent_id_] = asStopped;
  
  // TODO: time handling
  return 1.;
}

void ThreadedMultiAgentEnvironment::getAgentAction(int agent_id, Observation &obs, double reward, int terminal, Action *action)
{
  grl_assert(agent_id < state_.size());
  grl_assert(state_[agent_id] != asTerminated && state_[agent_id] != asStopped);

  ObservationRewardTerminal msg(obs, reward, terminal);
  
  if (terminal)
    state_[agent_id] = asTerminated;

  observation_[agent_id].write(msg);
  
  if (!terminal && action)
    *action = action_[agent_id].read();
}

void ThreadedMultiAgentEnvironment::prepareAgents(size_t agents)
{
  state_.resize(agents);
  
  observation_.clear();
  observation_.resize(agents);
  action_.clear();
  action_.resize(agents);

  for (size_t ii=0; ii < agents; ++ii)
    state_[ii] = asReady;
}

void ThreadedMultiAgentEnvironment::stopAgents()
{
  for (size_t ii=0; ii < state_.size(); ++ii)
  {
    while (state_[ii] == asReady) usleep(10);

    if (state_[ii] == asRunning)
    {
      state_[ii] = asTerminated;
      ObservationRewardTerminal msg(Observation(), 0, 2);
      observation_[ii].write(msg);
    }
    
    while (state_[ii] != asStopped) usleep(10);
  }
}
