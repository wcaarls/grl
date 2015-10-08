/** \file representation.h
 * \brief Generic mapping and representation definitions.
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

#ifndef GRL_REPRESENTATION_H_
#define GRL_REPRESENTATION_H_

#include <grl/configurable.h>
#include <grl/projection.h>
#include <grl/trace.h>

namespace grl
{

/// Maps inputs to outputs.
class Mapping : public Configurable
{
  public:
    virtual ~Mapping() { }
    virtual Mapping *clone() const = 0;

    /**
     * \brief Read out the mapping.
     *
     * Returns the value of the first output dimension.
     */
    virtual double read(const ProjectionPtr &projection, Vector *result) const = 0;
};

/// Approximates a Mapping.
class Representation : public Mapping
{
  public:
    virtual Representation *clone() const = 0;

    virtual double read(const ProjectionPtr &projection, Vector *result) const
    {
      return read(projection, result, NULL);
    }

    /// Read out the approximation.
    virtual double read(const ProjectionPtr &projection, Vector *result, Vector *stddev) const = 0;
    
    /// Add a new estimate of the target function, using a single learning rate for all outputs.
    virtual void write(const ProjectionPtr projection, const Vector &target, double alpha=1.)
    {
      Vector valpha;
      valpha = ConstantVector(target.size(), alpha);
      write(projection, target, valpha);
    }
    
    /// Add a new estimate of the target function, using separate learning rates per output.
    virtual void write(const ProjectionPtr projection, const Vector &target, const Vector &alpha) = 0;
    
    /// Adjust an estimate of the target function.
    virtual void update(const ProjectionPtr projection, const Vector &delta)
    {
      Vector value;
      read(projection, &value);
      write(projection, value+delta);
    }
    
    /// Update a trace of target function estimates
    virtual void update(const Trace &trace, const Vector &delta, double e=1.)
    {
      for (Trace::iterator ii=trace.begin(); ii != trace.end() && ii->weight() > 0.001; ++ii)
        update(ii->projection(), ii->weight()*delta*e);
    }
    
    /// Apply written values, if not already done on-line.
    virtual void finalize() { }
    
    /// Returns the Jacobian of the representation around the input projection.
    virtual Matrix jacobian(const ProjectionPtr projection) const
    {
      return Matrix();
    }
};

/// Representation that allows for parameter access.
class ParameterizedRepresentation : public Representation
{
  public:
    virtual ParameterizedRepresentation *clone() const = 0;
    
    /// Returns number of parameters.
    virtual size_t size() const = 0;
    
    /// Returns constant parameter vector.
    virtual const Vector &params() const = 0;
    
    /// Returns parameter vector.
    virtual Vector &params() = 0;
};

}

#endif /* GRL_REPRESENTATION_H_ */
