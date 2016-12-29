/** \file filtering.h
 * \brief Filtering agent header file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2016-07-20
 *
 * \copyright \verbatim
 * Copyright (c) 2016, Wouter Caarls
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

#ifndef GRL_FILTERING_AGENT_H_
#define GRL_FILTERING_AGENT_H_

#include <grl/agent.h>
#include <grl/policy.h>

namespace grl
{

/// Agent that filters incoming observations and outgoing actions.
class FilteringAgent : public Agent
{
  public:
    TYPEINFO("agent/filtering", "Agent that filters incoming observations and outgoing actions")

  protected:
    Agent *agent_;
    Vector observation_idx_, action_idx_, inv_action_idx_;
    int action_dims_;
    
  public:
    FilteringAgent() : agent_(NULL), action_dims_(0) { }
  
    // From Configurable    
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Agent
    virtual FilteringAgent *clone() const;
    virtual TransitionType start(const Vector &obs, Vector *action);
    virtual TransitionType step(double tau, const Vector &obs, double reward, Vector *action);
    virtual void end(double tau, const Vector &obs, double reward);
};

/// SubAgent that filters incoming observations and outgoing actions.
class FilteringSubAgent : public SubAgent
{
  public:
    TYPEINFO("agent/sub/filtering", "Subagent that filters incoming observations and outgoing actions")

  protected:
    SubAgent *agent_;
    Vector observation_idx_, action_idx_, inv_action_idx_;
    int action_dims_;
    
  public:
    FilteringSubAgent() : agent_(NULL), action_dims_(0) { }
  
    // From Configurable    
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From SubAgent
    virtual FilteringSubAgent *clone() const;
    virtual TransitionType start(const Vector &obs, Vector *action, double *conf);
    virtual TransitionType step(double tau, const Vector &obs, double reward, Vector *action, double *conf);
    virtual void end(double tau, const Vector &obs, double reward);
    virtual double confidence(const Vector &obs) const;
};

}

#endif /* GRL_FILTERING_AGENT_H_ */
