/** \file exclusive.h
 * \brief Exclusive master agent header file.
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

#ifndef GRL_EXCLUSIVE_MASTER_AGENT_H_
#define GRL_EXCLUSIVE_MASTER_AGENT_H_

#include <grl/predictor.h>
#include <grl/agent.h>

namespace grl
{

/// Fixed-policy agent.
class ExclusiveMasterAgent : public Agent
{
  public:
    TYPEINFO("agent/master/exclusive", "Master agent that selects one sub-agent to execute")

  protected:
    Predictor *predictor_;
    std::vector<SubAgent*> agent_;
    std::vector<double> time_;
    double gamma_, reward_;
    int last_agent_, smdp_steps_;
    Vector prev_obs_, prev_action_;
    
  public:
    ExclusiveMasterAgent() : predictor_(0), agent_(2), time_(2), gamma_(0.97), reward_(0), last_agent_(0), smdp_steps_(0)
    {
      agent_[0] = agent_[1] = NULL;
      time_[0] = time_[1] = -1;
    }
  
    // From Configurable    
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Agent
    virtual ExclusiveMasterAgent *clone() const;
    virtual void start(const Vector &obs, Vector *action);
    virtual void step(double tau, const Vector &obs, double reward, Vector *action);
    virtual void end(double tau, const Vector &obs, double reward);
};

}

#endif /* GRL_EXCLUSIVE_MASTER_AGENT_H_ */
