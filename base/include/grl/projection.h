/** \file projection.h
 * \brief Generic and basic projection definitions.
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

#ifndef GRL_PROJECTION_H_
#define GRL_PROJECTION_H_

#include <memory>

#include <grl/grl.h>

namespace grl
{

/// Stores information required to read out a Representation.
/**
 * Should support arithmetic required by Trace.
 */
class Projection
{
  public:
    virtual ~Projection() { }
    
    /// Saturated subtraction, used for replacing traces.
    virtual void ssub(const Projection &rhs) = 0;
};

typedef std::shared_ptr<Projection> ProjectionPtr;

/// Vector projection (e.g. basis function activations)
struct VectorProjection : public Projection
{
  LargeVector vector;

  virtual void ssub(const Projection &rhs)
  {
    const VectorProjection &vp = dynamic_cast<const VectorProjection&>(rhs);
    assert(vector.size() == vp.vector.size());
    for (size_t ii=0; ii < vector.size(); ++ii)
      vector[ii] = std::max(0., vector[ii]-vp.vector[ii]);
  }
};

/// Sparse boolean vector projection (e.g. TileCodingProjector)
struct IndexProjection : public Projection
{
  std::vector<size_t> indices;
  std::vector<double> weights;
  
  static size_t invalid_index()
  {
    return (size_t)-1; // maximum
  }

  /// Conversion to VectorProjection.
  virtual VectorProjection vector(size_t size)
  {
    VectorProjection vp;
    vp.vector = Vector::Zero(size);

    if (weights.empty())
      for (size_t ii=0; ii < indices.size(); ++ii)
        vp.vector[indices[ii]] = 1.0;
    else
      for (size_t ii=0; ii < indices.size(); ++ii)
        vp.vector[indices[ii]] = weights[ii];
        
    return vp;
  }
  
  virtual void ssub(const Projection &rhs)
  {
    const IndexProjection &ip = dynamic_cast<const IndexProjection&>(rhs);
    
    if (weights.empty() || ip.weights.empty())
    {
      for (size_t ii=0; ii < indices.size(); ++ii)
        for (size_t jj=0; jj < ip.indices.size(); ++jj)
          if (indices[ii] == ip.indices[jj])
            indices[ii] = invalid_index();
    }
    else
    {
      for (size_t ii=0; ii < indices.size(); ++ii)
        for (size_t jj=0; jj < ip.indices.size(); ++jj)
          if (indices[ii] == ip.indices[jj])
            weights[ii] = std::max(0., weights[ii]-ip.weights[jj]);
    }
  }
};

}

#endif /* GRL_PROJECTION_H_ */
