/*
 * projection.h
 *
 *  Created on: Jul 30, 2014
 *      Author: wcaarls
 */

#ifndef PROJECTION_H_
#define PROJECTION_H_

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
    virtual Projection *clone() const = 0;
    virtual void ssub(const Projection &rhs) = 0;
};

typedef boost::shared_ptr<Projection> ProjectionPtr;

/// Vector projection (e.g. basis function activations)
struct VectorProjection : public Projection
{
  Vector vector;

  virtual VectorProjection *clone() const
  {
    VectorProjection *vp = new VectorProjection();
    vp->vector = vector;
    return vp;
  }

  virtual void ssub(const Projection &rhs)
  {
    const VectorProjection &vp = dynamic_cast<const VectorProjection&>(rhs);
    assert(vector.size() == vp.vector.size());
    for (size_t ii=0; ii < vector.size(); ++ii)
      vector[ii] = fmax(0, vector[ii]-vp.vector[ii]);
  }
};

/// Sparse boolean vector projection (e.g. TileCodingProjector)
struct IndexProjection : public Projection
{
  std::vector<size_t> indices;
  
  static size_t invalid_index()
  {
    return (size_t)-1; // maximum
  }

  /// Conversion to VectorProjection.
  virtual VectorProjection vector(size_t size)
  {
    VectorProjection vp;
    vp.vector.resize(size);

    for (size_t ii=0; ii < indices.size(); ++ii)
      vp.vector[indices[ii]] = 1.0;

    return vp;
  }
  
  virtual IndexProjection *clone() const
  {
    IndexProjection *ip = new IndexProjection();
    ip->indices = indices;
    return ip;
  }

  virtual void ssub(const Projection &rhs)
  {
    const IndexProjection &ip = dynamic_cast<const IndexProjection&>(rhs);
    for (size_t ii=0; ii < indices.size(); ++ii)
      for (size_t jj=0; jj < ip.indices.size(); ++jj)
        if (indices[ii] == ip.indices[ii])
          indices[ii] = invalid_index();
  }
};

}

#endif /* PROJECTION_H_ */
