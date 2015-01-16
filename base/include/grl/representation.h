/*
 * representation.h
 *
 *  Created on: Jul 30, 2014
 *      Author: wcaarls
 */

#ifndef REPRESENTATION_H_
#define REPRESENTATION_H_

#include <grl/configurable.h>
#include <grl/projection.h>
#include <grl/trace.h>

namespace grl
{

/// Maps some inputs to a RepresentedQuantity.
class Mapping : public Configurable
{
  protected:
    RepresentedQuantity rq_;

  public:
    virtual ~Mapping() { }
    virtual Mapping *clone() const = 0;

    RepresentedQuantity quantity() const { return rq_; }
    virtual double read(const ProjectionPtr &projection, Vector *result) const = 0;
};

/// Approximates a Mapping.
class Representation : public Mapping
{
  public:
    virtual Representation *clone() const = 0;

    virtual void write(const ProjectionPtr projection, const Vector &target, double alpha=1.) = 0;
    virtual void update(const ProjectionPtr projection, const Vector &delta)
    {
      Vector value;
      read(projection, &value);
      write(projection, value+delta);
    }
    virtual void update(const Trace &trace, const Vector &delta)
    {
      for (Trace::iterator ii=trace.begin(); ii != trace.end() && ii->weight() > 0.001; ++ii)
        update(ii->projection(), ii->weight()*delta);
    }
};

/// Representation that allows for parameter access.
class ParameterizedRepresentation : public Representation
{
  protected:
    Vector params_;

  public:
    virtual ParameterizedRepresentation *clone() const = 0;
    virtual size_t size() const = 0;

    virtual const Vector &params() const
    {
      return params_;
    }
    
    virtual Vector &params()
    {
      return params_;
    }
};

}

#endif /* REPRESENTATION_H_ */
