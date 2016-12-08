/** \file voluntary.h
 * \brief Voluntary sub-agent header file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2016-02-03
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

#ifndef GRL_VOLUNTARY_AGENT_H_
#define GRL_VOLUNTARY_AGENT_H_

#include <grl/agent.h>

namespace grl
{

/// Fixed-policy agent.
class VoluntarySubAgent : public SubAgent
{
  public:
    TYPEINFO("agent/sub/voluntary", "Sub agent that has confidence as part of the action")

  protected:
    Agent *agent_;
    size_t dim_;
    
  public:
    VoluntarySubAgent() : agent_(NULL), dim_(0) { }
  
    // From Configurable    
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Agent
    virtual void end(double tau, const Vector &obs, double reward);
    
    // From SubAgent
    virtual void start(const Vector &obs, Vector *action, double *confidence);
    virtual void step(double tau, const Vector &obs, double reward, Vector *action, double *confidence);
    double confidence(const Vector &obs) const
    {
      ERROR("Cannot determine confidence without executing agent");
      return 0.;
    }
};

}

#endif /* GRL_VOLUNTARY_AGENT_H_ */
