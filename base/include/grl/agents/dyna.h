/** \file dyna.h
 * \brief Dyna agent header file.
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

#ifndef GRL_DYNA_AGENT_H_
#define GRL_DYNA_AGENT_H_

#include <itc/itc.h>

#include <grl/agent.h>
#include <grl/policy.h>
#include <grl/predictors/model.h>
#include <grl/environments/observation.h>

namespace grl
{

/// Dyna model-based learning agent.
class DynaAgent : public Agent, public itc::Thread
{
  public:
    TYPEINFO("agent/dyna", "Agent that learns from both observed and predicted state transitions")

  protected:
    Policy *policy_;
    Predictor *predictor_;
    
    ObservationModel *model_;
    ModelPredictor *model_predictor_;
    Agent *model_agent_;
    State *state_;
    
    Vector start_obs_, prev_obs_, prev_action_;
    size_t planning_steps_, planning_horizon_, total_planned_steps_;
    
    double planning_reward_, actual_reward_;
    size_t planned_steps_, control_steps_, total_control_steps_;
    int asynchronous_;
    
    double time_;
    
  public:
    DynaAgent() : policy_(NULL), predictor_(NULL), model_(NULL), model_predictor_(NULL), model_agent_(NULL), state_(NULL), planning_steps_(1), planning_horizon_(100), total_planned_steps_(0), planning_reward_(0.), actual_reward_(0.), planned_steps_(0), control_steps_(0), total_control_steps_(0), asynchronous_(0), time_(0.) { }
  
    // From itc::Thread
    using itc::Thread::start;

    // From Configurable    
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Agent
    virtual DynaAgent *clone() const;
    virtual void start(const Vector &obs, Vector *action);
    virtual void step(double tau, const Vector &obs, double reward, Vector *action);
    virtual void end(double tau, double reward);
    virtual void report(std::ostream &os);
    
  protected:
    virtual void run();
    
    void runModel();
};

}

#endif /* GRL_DYNA_AGENT_H_ */
