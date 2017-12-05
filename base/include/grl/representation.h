/** \file representation.h
 * \brief Generic representation definitions.
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

#include <pthread.h>

#include <grl/configurable.h>
#include <grl/projection.h>
#include <grl/trace.h>

namespace grl
{

/// Approximates a Mapping.
class Representation : public Configurable
{
  private:
    std::vector<ProjectionPtr> batch_;
    pthread_mutex_t mutex_;

  public:
    Representation() : mutex_(PTHREAD_MUTEX_INITIALIZER) { }
  
    /// Read out the approximation.
    virtual double read(const ProjectionPtr &projection, Vector *result) const
    {
      return read(projection, result, NULL);
    }

    /// Read out the approximation, returning standard deviation of result.
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

    /// Starts a batch read operation.
    virtual void batchRead(size_t sz)
    {
      // Make sure other threads can't interrupt our batch
      pthread_mutex_lock(&mutex_);
    
      batch_.clear();
      batch_.reserve(sz);
    }

    /// Starts a batch write operation.
    virtual void batchWrite(size_t sz)
    {
    }
    
    /// Enqueues a batch read operation, to be read later.
    virtual void enqueue(const ProjectionPtr &projection)
    {
      batch_.push_back(projection);
    }

    /// Enqueues a batch write operation, to be executed later.
    virtual void enqueue(const ProjectionPtr &projection, const Vector &target)
    {
      // Assume write is not reentrant for regular representations
      write(projection, target);
    }
    
    /// Reads the results of all enqueued read operations.
    virtual void read(Matrix *out)
    {
      Vector result;
      read(batch_[0], &result);
      if (!result.size())
      {
        *out = Matrix();
        return;
      }
      
      *out = Matrix(batch_.size(), result.size());
      out->row(0) = result;
      
      #ifdef _OPENMP
      #pragma omp parallel for
      #endif
      for (size_t ii=1; ii < batch_.size(); ++ii)
      {
        Vector result;
        read(batch_[ii], &result);
        out->row(ii) = result;
      }

      pthread_mutex_unlock(&mutex_);
    }
    
    /// Executes all enqueued write operations.
    virtual void write()
    {
      finalize();

      pthread_mutex_unlock(&mutex_);
    }
    
    /// Returns the representation to use when calculating targets.
    virtual Representation *target()
    {
      return this;
    }
};

/// Representation that allows for parameter access.
class ParameterizedRepresentation : public Representation
{
  public:
    /// Returns number of parameters.
    virtual size_t size() const = 0;
    
    /// Returns constant parameter vector.
    virtual const LargeVector &params() const = 0;
    
    /// Sets parameter vector.
    virtual void setParams(const LargeVector &params) = 0;
};

}

#endif /* GRL_REPRESENTATION_H_ */
