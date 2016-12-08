/** \file voluntary.cpp
 * \brief Voluntary sub-agent source file.
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

#include <grl/agents/voluntary.h>

using namespace grl;

REGISTER_CONFIGURABLE(VoluntarySubAgent)

void VoluntarySubAgent::request(ConfigurationRequest *config)
{
  config->push_back(CRP("dim", "Action dimension that indicates confidence", (int)dim_));
  config->push_back(CRP("agent", "agent", "Sub agent", agent_));
}

void VoluntarySubAgent::configure(Configuration &config)
{
  agent_ = (Agent*)config["agent"].ptr();

  dim_ = config["dim"];
}

void VoluntarySubAgent::reconfigure(const Configuration &config)
{
}

void VoluntarySubAgent::start(const Vector &obs, Vector *action, double *confidence)
{
  Vector a;
  agent_->start(obs, &a);
  
  if (dim_ >= a.size())
    throw bad_param("agent/sub/voluntary:dim");

  *confidence = a[dim_];
  
  // Remove indicator dimension
  *action = Vector(a.size()-1);
  *action << a.leftCols(dim_), a.rightCols(a.size()-dim_-1);
}

void VoluntarySubAgent::step(double tau, const Vector &obs, double reward, Vector *action, double *confidence)
{
  Vector a;
  agent_->step(tau, obs, reward, &a);

  if (dim_ >= a.size())
    throw bad_param("agent/sub/voluntary:dim");

  *confidence = a[dim_];
  
  // Remove indicator dimension
  *action = Vector(a.size()-1);
  *action << a.leftCols(dim_), a.rightCols(a.size()-dim_-1);
}

void VoluntarySubAgent::end(double tau, const Vector &obs, double reward)
{
  agent_->end(tau, obs, reward);
}
