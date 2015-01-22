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

namespace grl
{

/// Projects a state onto a Projection.
class Projector : public Configurable
{
  public:
    virtual ~Projector() { }
    virtual Projector *clone() const = 0;
    virtual ProjectionPtr project(const Vector &in) const = 0;
    virtual ProjectionPtr project(const Vector &base, const Vector &variant) const
    {
      Vector v = base;
      v.insert(v.end(), variant.begin(), variant.end());
      return project(v);
    }

    virtual void project(const Vector &base, const std::vector<Vector> &variants, std::vector<ProjectionPtr> *out) const
    {
      Vector v = base;

      for (size_t ii=0; ii < variants.size(); ++ii)
      {
        v.insert(v.end(), variants[ii].begin(), variants[ii].end());
        out->push_back(project(v));
        v.erase(v.end()-variants[ii].size(), v.end());
      }
    }
};

/// Simply returns the input vector (for e.g. \link DeterministicActionPolicy DeterministicActionPolicies \endlink)
class IdentityProjector : public Projector
{
  public:
    TYPEINFO("projector/identity")

  public:
    virtual IdentityProjector *clone() const
    {
      return new IdentityProjector();
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
    TYPEINFO("projector/sample")

  public:
    virtual void push(class Sample *sample) = 0;
};

}

#endif /* GRL_PROJECTOR_H_ */
