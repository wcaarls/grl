/** \file delayed_td.h
 * \brief Delayed temporal difference agent header file.
 *
 * \author    Ivan Koryakovskiy <i.koryakovskiy@google.com>
 * \date      2017-02-12
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

#ifndef GRL_DELAYED_TD_AGENT_H_
#define GRL_DELAYED_TD_AGENT_H_

#include <grl/agent.h>
#include <grl/policy.h>
#include <grl/predictor.h>
#include <grl/mutex.h>
#include <grl/signal.h>

namespace grl
{

/// Temporal difference learning agent.
class DelayedTDAgent : public Agent
{
  public:
    TYPEINFO("agent/delayed_td", "Agent that learns from observed state transitions assuming non-integer values of control delay")

    struct DelayedTDAgentState
    {
      double time;
      Observation prev_obs;
      Action prev_action;
      Action prev_prev_action;
    };

  protected:
    Policy *policy_;
    Predictor *predictor_;
    double control_delay_;
    
    Instance<DelayedTDAgentState> agent_state_;
    
  public:
    DelayedTDAgent() : policy_(NULL), predictor_(NULL), control_delay_(0) { }
  
    // From Configurable    
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Agent
    virtual void start(const Observation &obs, Action *action);
    virtual void step(double tau, const Observation &obs, double reward, Action *action);
    virtual void end(double tau, const Observation &obs, double reward);

  protected:
    virtual Action combine(const Action &a0, const Action &a1) const;
};

}

#endif /* GRL_DELAYED_TD_AGENT_H_ */
