/** \file sequential.h
 * \brief Sequential master agent header file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-09-10
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

#ifndef GRL_SEQUENTIAL_MASTER_AGENT_H_
#define GRL_SEQUENTIAL_MASTER_AGENT_H_

#include <grl/agent.h>
#include <grl/predictor.h>
#include <grl/exporter.h>

namespace grl
{

/// Fixed-policy agent.
class SequentialMasterAgent : public Agent
{
  public:
    TYPEINFO("agent/master/sequential", "Master agent that executes sub-agents sequentially")

  protected:
    Predictor *predictor_;
    std::vector<Agent*> agent_;
    Exporter *exporter_;
    double time_;
    Vector prev_obs_, prev_action_;
    
  public:
    SequentialMasterAgent() : predictor_(0), agent_(2)
    {
      agent_[0] = agent_[1] = NULL;
    }
  
    // From Configurable    
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Agent
    virtual SequentialMasterAgent *clone() const;
    virtual void start(const Vector &obs, Vector *action);
    virtual void step(double tau, const Vector &obs, double reward, Vector *action);
    virtual void end(double tau, const Vector &obs, double reward);
};

/// Fixed-policy agent.
class SequentialAdditiveMasterAgent : public SequentialMasterAgent
{
  public:
    TYPEINFO("agent/master/sequential/additive", "Additive master agent that executes sub-agents sequentially and adds their outputs")

  protected:
    Vector min_, max_;

  public:

    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Agent
    virtual SequentialAdditiveMasterAgent *clone() const;
    virtual void start(const Vector &obs, Vector *action);
    virtual void step(double tau, const Vector &obs, double reward, Vector *action);
    virtual void end(double tau, const Vector &obs, double reward);
};

}

#endif /* GRL_SEQUENTIAL_MASTER_AGENT_H_ */
