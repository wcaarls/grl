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

#include <grl/agent.h>
#include <grl/policy.h>
#include <grl/predictor.h>
#include <grl/projector.h>
#include <grl/representation.h>

namespace grl
{

/// Dyna model-based learning agent.
class DynaAgent : public Agent
{
  public:
    TYPEINFO("agent/dyna")

  protected:
    Policy *policy_;
    Predictor *predictor_;
    Agent *model_agent_;
    Projector *model_projector_;
    Representation *model_representation_;
    
    Vector start_obs_, prev_obs_, prev_action_, wrapping_;
    size_t planning_steps_;
    Vector observation_min_, observation_max_;
    
  public:
    DynaAgent() : policy_(NULL), predictor_(NULL), model_agent_(NULL), model_projector_(NULL), model_representation_(NULL), planning_steps_(1) { }
  
    // From Configurable    
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Agent
    virtual DynaAgent *clone() const;
    virtual void start(const Vector &obs, Vector *action);
    virtual void step(const Vector &obs, double reward, Vector *action);
    virtual void end(double reward);
    
  protected:
    void learnModel(const Transition &t);
    void stepModel(const Vector &obs, const Vector &action, Vector *next, double *reward, int *terminal);
    void runModel();
  
};

}

#endif /* GRL_DYNA_AGENT_H_ */
