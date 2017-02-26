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

REGISTER_CONFIGURABLE(GridIndexProjector)
REGISTER_CONFIGURABLE(GridPositionProjector)

void GridProjector::request(const std::string &role, ConfigurationRequest *config)
{
  if (role.size())
    config->push_back(CRP("discretizer", "discretizer." + role, "Discretizer", discretizer_));
  else
    config->push_back(CRP("discretizer", "discretizer", "Discretizer", discretizer_));
  
  config->push_back(CRP("memory", "int.memory", "Grid size", CRP::Provided));
}

void GridProjector::configure(Configuration &config)
{
  discretizer_ = (Discretizer*)config["discretizer"].ptr();
  
  config.set("memory", discretizer_->size());
}

void GridProjector::reconfigure(const Configuration &config)
{
}

ProjectionPtr GridIndexProjector::project(const Vector &in) const
{
  IndexProjection *p = new IndexProjection();
  p->indices.resize(1);
  
  p->indices[0] = discretizer_->discretize(in);

  return ProjectionPtr(p);
}

ProjectionPtr GridPositionProjector::project(const Vector &in) const
{
  VectorProjection *p = new VectorProjection();
  p->vector = discretizer_->at(discretizer_->discretize(in));

  return ProjectionPtr(p);
}
