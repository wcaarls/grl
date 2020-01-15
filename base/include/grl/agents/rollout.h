/** \file rollout.h
 * \brief Policy rollout agent header file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2020-01-15
 *
 * \copyright \verbatim
 * Copyright (c) 2020, Wouter Caarls
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

#ifndef GRL_ROLLOUT_AGENT_H_
#define GRL_ROLLOUT_AGENT_H_

#include <grl/agent.h>
#include <grl/policy.h>
#include <grl/predictor.h>
#include <grl/mutex.h>
#include <grl/signal.h>

namespace grl
{

/// Policy rollout agent.
class RolloutAgent : public Agent
{
  public:
    TYPEINFO("agent/rollout", "Agent that learns from policy rollouts")

  protected:
    Policy *policy_;
    Predictor *predictor_;
    int steps_; 

    double time_;
    Observation prev_obs_;
    Action prev_action_;
    
    std::vector<const Transition*> transitions_;
    
  public:
    RolloutAgent() : policy_(NULL), predictor_(NULL), steps_(1000) { }
  
    // From Configurable    
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Agent
    virtual void start(const Observation &obs, Action *action);
    virtual void step(double tau, const Observation &obs, double reward, Action *action);
    virtual void end(double tau, const Observation &obs, double reward);
    
  protected:
    void addTransition(const Observation &prev_obs, const Action &prev_action, double tau, double reward, const Observation &obs, const Action &action=Action());
};

}

#endif /* GRL_ROLLOUT_AGENT_H_ */
