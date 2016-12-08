/** \file multi.cpp
 * \brief Multi projector source file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2016-12-06
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

#include <string.h>
#include <grl/projectors/multi.h>

using namespace grl;

REGISTER_CONFIGURABLE(MultiProjector)

void MultiProjector::request(const std::string &role, ConfigurationRequest *config)
{
  config->push_back(CRP("projector1", "projector." + role, "First downstream projector", projector_[0]));
  config->push_back(CRP("projector2", "projector." + role, "Second downstream projector", projector_[1]));

  config->push_back(CRP("memories", "vector.memory", "Memory of downstream projectors", memory_));
  config->push_back(CRP("memory", "int.memory", "Feature vector size", CRP::Provided));
}

void MultiProjector::configure(Configuration &config)
{
  memory_ = config["memories"].v();

  projector_[0] = (Projector*) config["projector1"].ptr();
  projector_[1] = (Projector*) config["projector2"].ptr();
  
  config.set("memory", (int)sum(memory_));
}

void MultiProjector::reconfigure(const Configuration &config)
{
}

ProjectionPtr MultiProjector::project(const Vector &in) const
{
  ProjectionPtr p = projector_[0]->project(in);
  IndexProjection *ip = dynamic_cast<IndexProjection*>(p.get());
  
  if (ip)
  {
    size_t memory = memory_[0];
  
    for (size_t ii=1; ii < projector_.size(); ++ii)
    {
      ProjectionPtr p2 = projector_[ii]->project(in);
      IndexProjection *ip2 = dynamic_cast<IndexProjection*>(p2.get());
      if (!ip2)
        throw Exception("projector/multi requires consistent downstream projectors");

      ip->indices.reserve(ip->indices.size() + ip2->indices.size());
      for (size_t jj=0; jj < ip2->indices.size(); ++jj)
        ip->indices.push_back(memory + ip2->indices[jj]);
        
      ip->weights.reserve(ip->weights.size() + ip2->weights.size());
      for (size_t jj=0; jj < ip2->weights.size(); ++jj)
        ip->weights.push_back(ip2->weights[jj]);
        
      memory += memory_[ii];
    }
  }
  else
  {
    VectorProjection *vp = dynamic_cast<VectorProjection*>(p.get());
    if (vp)
    {
      for (size_t ii=1; ii < projector_.size(); ++ii)
      {
        ProjectionPtr p2 = projector_[ii]->project(in);
        VectorProjection *vp2 = dynamic_cast<VectorProjection*>(p2.get());
        if (!vp2)
          throw Exception("projector/multi requires consistent downstream projectors");
          
        vp->vector = extend(vp->vector, vp2->vector);
      }
    }
    else
      throw Exception("projector/multi requires a downstream projectors returning IndexProjection or VectorProjection");
  }
  
  return p;
}
