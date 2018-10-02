/** \file replay.h
 * \brief Experience replay agent header file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2017-07-14
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

#ifndef GRL_REPLAY_AGENT_H_
#define GRL_REPLAY_AGENT_H_

#include <itc/itc.h>

#include <grl/agent.h>
#include <grl/policy.h>
#include <grl/predictors/model.h>
#include <grl/environments/observation.h>

namespace grl
{

class ReplayAgentThread : public itc::Thread
{
  protected:
    class ReplayAgent *agent_;
    
  public:
    ReplayAgentThread(class ReplayAgent *agent) : agent_(agent) { }
    
  protected:
    virtual void run();
};

/// Experience replay agent.
class ReplayAgent : public Agent
{
  friend class ReplayAgentThread;

  public:
    TYPEINFO("agent/replay", "Agent that learns from batches of stored transitions")

  protected:
    Policy *policy_;
    Predictor *predictor_;
    int memory_size_, replay_steps_, batch_size_, observation_steps_;
    int threads_;

    std::vector<ReplayAgentThread*> replay_threads_;
    std::vector<Transition> transitions_;
    
    double time_;
    Observation prev_obs_;
    Action prev_action_;
    size_t total_control_steps_, total_replay_steps_, total_transitions_;
    
  public:
    ReplayAgent() : policy_(NULL), predictor_(NULL), memory_size_(100000), replay_steps_(1), batch_size_(1), observation_steps_(1), threads_(0), time_(0.), total_control_steps_(0), total_replay_steps_(0), total_transitions_(0) { }
    ~ReplayAgent()
    {
      stopThreads();
    }
  
    // From Configurable    
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Agent
    virtual void start(const Observation &obs, Action *action);
    virtual void step(double tau, const Observation &obs, double reward, Action *action);
    virtual void end(double tau, const Observation &obs, double reward);
    virtual void report(std::ostream &os);

  protected:
    void replay();
    void startThreads();
    void stopThreads();
};

}

#endif /* GRL_REPLAY_AGENT_H_ */
