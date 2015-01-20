/*
 * projector.h
 *
 *  Created on: Jul 30, 2014
 *      Author: wcaarls
 */

#ifndef PROJECTOR_H_
#define PROJECTOR_H_

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

class NeighborProjector : public Projector
{
  public:
    TYPEINFO("projector/neighbor")

  public:
    virtual void push(class Sample *sample) = 0;
};

}

#endif /* PROJECTOR_H_ */
