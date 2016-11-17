/** \file td.h
 * \brief Temporal difference agent header file.
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

#ifndef GRL_TD_AGENT_H_
#define GRL_TD_AGENT_H_

#include <grl/agent.h>
#include <grl/policy.h>
#include <grl/predictor.h>
#include <grl/mutex.h>
#include <grl/signals/signal_v.h>

namespace grl
{

/// Temporal difference learning agent.
class TDAgent : public Agent
{
  public:
    TYPEINFO("agent/td", "Agent that learns from observed state transitions")

    struct TDAgentState
    {
      double time;
      Vector prev_obs, prev_action;
    };

  protected:
    Policy *policy_;
    Predictor *predictor_;
    
    Instance<TDAgentState> agent_state_;
    
    Signal *transition_type_;

  public:
    TDAgent() : policy_(NULL), predictor_(NULL), transition_type_(NULL) { }
  
    // From Configurable    
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Agent
    virtual TDAgent *clone() const;
    virtual void start(const Vector &obs, Vector *action);
    virtual void step(double tau, const Vector &obs, double reward, Vector *action);
    virtual void end(double tau, const Vector &obs, double reward);
};

}

#endif /* GRL_TD_AGENT_H_ */
