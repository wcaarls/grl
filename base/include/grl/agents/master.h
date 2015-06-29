/** \file master.h
 * \brief Master agent header file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-06-16
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

#ifndef GRL_MASTER_AGENT_H_
#define GRL_MASTER_AGENT_H_

#include <grl/agent.h>

namespace grl
{

/// Fixed-policy agent.
class MasterAgent : public Agent
{
  public:
    TYPEINFO("agent/master", "Master agent that chooses among sub-agents")

  protected:
    std::vector<SubAgent*> agent_;
    std::vector<bool> started_;
    double gamma_, reward_;
    int last_agent_, smdp_steps_;
    
  public:
    MasterAgent() : agent_(2), started_(2), gamma_(0.97), reward_(0), last_agent_(-1), smdp_steps_(0)
    {
      agent_[0] = agent_[1] = NULL;
      started_[0] = started_[1] = false;
    }
  
    // From Configurable    
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Agent
    virtual MasterAgent *clone() const;
    virtual void start(const Vector &obs, Vector *action);
    virtual void step(const Vector &obs, double reward, Vector *action);
    virtual void end(double reward);
};

}

#endif /* GRL_MASTER_AGENT_H_ */
