/** \file split.cpp
 * \brief Split projector source file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2017-02-09
 *
 * \copyright \verbatim
 * Copyright (c) 2017, Wouter Caarls
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
#include <grl/projectors/split.h>

using namespace grl;

REGISTER_CONFIGURABLE(SplitProjector)

void SplitProjector::request(const std::string &role, ConfigurationRequest *config)
{
  config->push_back(CRP("index", "vector", "Binary vector that specifies which dimensions to use as index", index_));
  config->push_back(CRP("discretizer", "discretizer", "Determines the distinct set based on the index dimensions", discretizer_));
  config->push_back(CRP("projector", "projector", "Projects the non-index dimensions onto a feature vector", projector_));

  config->push_back(CRP("projector_memory", "int.memory", "Memory of downstream projector", (int)projector_memory_, CRP::System));
  config->push_back(CRP("memory", "int.memory", "Resulting feature vector size", CRP::Provided));
}

void SplitProjector::configure(Configuration &config)
{
  index_ = config["index"].v();
  projector_memory_ = config["projector_memory"];
  discretizer_ = (Discretizer*) config["discretizer"].ptr();
  projector_ = (Projector*) config["projector"].ptr();
  
  if (!projector_memory_)
    throw bad_param("projector/split:projector_memory");
  
  TRACE("Projecting into " << discretizer_->size() << " distinct sets of " << projector_memory_ << " features each");
  
  config.set("memory", discretizer_->size() * projector_memory_);
}

void SplitProjector::reconfigure(const Configuration &config)
{
}

ProjectionPtr SplitProjector::project(const Vector &in) const
{
  if (in.size() != index_.size())
    throw bad_param("projector/split:index");

  // Extract index dimensions
  Vector idx((size_t)sum(index_));
  Vector v(index_.size()-idx.size());
  
  for (size_t ii=0, jj=0; ii < index_.size(); ++ii)
    if (index_[ii])
      idx[jj++] = in[ii];
    else
      v[ii-jj] = in[ii];
      
  // Find offset
  size_t offset = discretizer_->discretize(idx)*projector_memory_;

  // Project remaining dimensions
  ProjectionPtr pp = projector_->project(v);
  
  VectorProjection *vp = dynamic_cast<VectorProjection*>(pp.get());
  if (vp)
  {
    // Insert feature vector at offset position
    VectorProjection *p = new VectorProjection;
    p->vector = LargeVector::Zero(discretizer_->size()*projector_memory_);
    p->vector.segment(offset, projector_memory_) = vp->vector;
    return ProjectionPtr(p);
  }
  else
  {
    IndexProjection *ip = dynamic_cast<IndexProjection*>(pp.get());
    if (ip)
    {
      // Add offset to indices
      for (size_t ii=0; ii < ip->indices.size(); ++ii)
        ip->indices[ii] += offset;
        
      return pp;
    }
    else
      throw Exception("projector/split requires a downstream projector returning IndexProjection or VectorProjection");
  }
}
