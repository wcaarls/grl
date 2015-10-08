/** \file grid.cpp
 * \brief Grid projector source file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-08-28
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

#include <grl/projectors/grid.h>

using namespace grl;

REGISTER_CONFIGURABLE(GridProjector)

void GridProjector::request(const std::string &role, ConfigurationRequest *config)
{
  if (role == "observation")
  {
    config->push_back(CRP("input_min", "vector.observation_min", "Lower input dimension limit", min_, CRP::System));
    config->push_back(CRP("input_max", "vector.observation_max", "Upper input dimension limit", max_, CRP::System));
  }
  else if (role == "action")
  {
    config->push_back(CRP("input_min", "vector.action_min", "Lower input dimension limit", min_, CRP::System));
    config->push_back(CRP("input_max", "vector.action_max", "Upper input dimension limit", max_, CRP::System));
  }
  else if (role == "pair")
  {
    config->push_back(CRP("input_min", "vector.observation_min+vector.action_min", "Lower input dimension limit", min_, CRP::System));
    config->push_back(CRP("input_max", "vector.observation_max+vector.action_max", "Upper input dimension limit", max_, CRP::System));
  }
  else
  {
    config->push_back(CRP("input_min", "Lower input dimension limit", min_, CRP::System));
    config->push_back(CRP("input_max", "Upper input dimension limit", max_, CRP::System));
  }

  config->push_back(CRP("steps", "Grid cells per dimension", steps_, CRP::Configuration));
  config->push_back(CRP("memory", "int.memory", "Grid size", CRP::Provided));
}

void GridProjector::configure(Configuration &config)
{
  min_ = config["input_min"];
  if (!min_.size())
    throw bad_param("projector/grid:min");

  max_ = config["input_max"];
  if (!max_.size())
    throw bad_param("projector/grid:max");

  steps_ = config["steps"];
  if (!steps_.size())
    throw bad_param("projector/grid:steps");

  if (min_.size() != max_.size() || min_.size() != steps_.size())
    throw bad_param("projector/grid:{min,max,steps}");

  delta_ = (max_-min_)/steps_;
  
  config.set("memory", (int)prod(steps_));
}

void GridProjector::reconfigure(const Configuration &config)
{
}

GridProjector *GridProjector::clone() const
{
  return new GridProjector(*this);
}

ProjectionPtr GridProjector::project(const Vector &in) const
{
  if (in.size() != min_.size())
    throw bad_param("projector/grid:{min,max,steps}");

  IndexProjection *p = new IndexProjection();
  p->indices.resize(1);
  
  size_t index = 0;

  size_t ff = 1;
  for (size_t dd=0; dd < steps_.size(); ++dd)
  {
    size_t v = std::min(std::max((in[dd]-min_[dd])/delta_[dd], 0.), steps_[dd]-1.);
    index += ff*v;
    ff *= steps_[dd];
  }

  p->indices[0] = index;
  return ProjectionPtr(p);
}
