/** \file dsp.h
 * \brief DSP agent header file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2022-05-07
 *
 * \copyright \verbatim
 * Copyright (c) 2022, Wouter Caarls
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

#ifndef GRL_DSP_AGENT_H_
#define GRL_DSP_AGENT_H_

#include <grl/agent.h>
#include <grl/filter.h>
#include <grl/signal.h>

namespace grl
{

/// Agent that filters inputs and outputs using DSP filter
class DSPAgent : public Agent
{
  public:
    TYPEINFO("agent/dsp", "DSP filtering agent")

  protected:
    Agent *agent_;
    TypedConfigurableList<Filter> input_filters_;
    TypedConfigurableList<Filter> output_filters_;
    
    VectorSignal *state_, *action_;
    
  public:
    DSPAgent() : agent_(NULL), state_(NULL), action_(NULL) { }
  
    // From Configurable    
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Agent
    virtual void start(const Observation &obs, Action *action);
    virtual void step(double tau, const Observation &obs, double reward, Action *action);
    virtual void end(double tau, const Observation &obs, double reward);
    
  protected:
    /// Apply a filter stack to a sample
    Vector apply(TypedConfigurableList<Filter> &filters, Vector sample, bool reset=false);
};

}

#endif /* GRL_DSP_AGENT_H_ */
