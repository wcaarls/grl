/** \file projector.h
 * \brief Generic and basic projector defintions.
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

#ifndef GRL_PROJECTOR_H_
#define GRL_PROJECTOR_H_

#include <grl/configurable.h>
#include <grl/projection.h>
#include <grl/projections/sample.h>

namespace grl
{

/// Projects a state onto a Projection.
class Projector : public Configurable
{
  public:
    /// Lifetime of a Projection.
    enum ProjectionLifetime { plIndefinite, ///< Always valid.
                              plWrite,      ///< Valid until a new projection is written to.
                              plUpdate      ///< Valid until a projection is updated.
                            };

  public:
    virtual ~Projector() { }
    
    /// Retrieves the lifetime of Projections made by this Projector.
    virtual ProjectionLifetime lifetime() const = 0;
    
    /// Project a single input.
    virtual ProjectionPtr project(const Vector &in) const = 0;
    
    /// Project a single input, provided as a base and a variant (e.g. state and action).
    virtual ProjectionPtr project(const Vector &base, const Vector &variant) const
    {
      return project(extend(base, variant));
    }

    /// Project a set of inputs, provided as a base and a set of variants (e.g. state and possible actions).
    virtual void project(const Vector &base, const std::vector<Vector> &variants, std::vector<ProjectionPtr> *out) const
    {
      out->clear();
      for (size_t ii=0; ii < variants.size(); ++ii)
        out->push_back(project(extend(base, variants[ii])));
    }
    
    /// Returns the Jacobian of the projection around the input vector.
    virtual Matrix jacobian(const Vector &in) const
    {
      return Matrix::Identity(in.size(), in.size());
    }
};

/// Simply returns the input vector (for e.g. \link ActionPolicy ActionPolicies \endlink)
class IdentityProjector : public Projector
{
  public:
    TYPEINFO("projector/identity", "Simply returns the input vector")

  public:
    virtual ProjectionLifetime lifetime() const
    {
      return plIndefinite;
    }
    
    virtual ProjectionPtr project(const Vector &in) const
    {
      VectorProjection *vp = new VectorProjection();
      vp->vector = in;
      return ProjectionPtr(vp);
    }
};

/// Projects onto a changing set of samples.
class SampleProjector : public Projector
{
  public:
    virtual void push(class Sample *sample) = 0;
    virtual StorePtr store() = 0;
    
    /// Add pushed samples to store.
    virtual void finalize() = 0;
};

}

#endif /* GRL_PROJECTOR_H_ */
