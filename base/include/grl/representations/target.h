/** \file target.h
 * \brief Target representation header file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2017-07-13
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

#ifndef GRL_TARGET_REPRESENTATION_H_
#define GRL_TARGET_REPRESENTATION_H_

#include <grl/representation.h>

namespace grl
{

/// Representation that periodically updates a target representation.
class TargetRepresentation : public Representation
{
  public:
    TYPEINFO("representation/target", "Representation that periodically updates a target representation")

  protected:
    ParameterizedRepresentation* representation_, *target_representation_;
    double interval_;
    int count_;
    
  public:
    TargetRepresentation() : representation_(NULL), target_representation_(NULL), interval_(100)
    {
    }
    
    // From Configurable
    virtual void request(const std::string &role, ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);
  
    // From Representation
    virtual double read(const ProjectionPtr &projection, Vector *result) const
    {
      return representation_->read(projection, result);
    }
    
    virtual double read(const ProjectionPtr &projection, Vector *result, Vector *stddev) const
    {
      return representation_->read(projection, result, stddev);
    }
    
    virtual void write(const ProjectionPtr projection, const Vector &target, double alpha=1.)
    {
      representation_->write(projection, target, alpha);
      checkSynchronize();
    }
    
    virtual void write(const ProjectionPtr projection, const Vector &target, const Vector &alpha)
    {
      representation_->write(projection, target, alpha);
      checkSynchronize();
    }
    
    virtual void update(const ProjectionPtr projection, const Vector &delta)
    {
      representation_->update(projection, delta);
    }
    
    virtual void update(const Trace &trace, const Vector &delta, double e=1.)
    {
      representation_->update(trace, delta, e);
    }
    
    virtual void finalize()
    {
      representation_->finalize();
    }
    
    virtual Matrix jacobian(const ProjectionPtr projection) const
    {
      return representation_->jacobian(projection);
    }
    
    virtual void batchRead(size_t sz)
    {
      representation_->batchRead(sz);
    }
    
    virtual void batchWrite(size_t sz)
    {
      representation_->batchWrite(sz);
    }
    
    virtual void enqueue(const ProjectionPtr &projection)
    {
      representation_->enqueue(projection);
    }
    
    virtual void enqueue(const ProjectionPtr &projection, const Vector &target)
    {
      representation_->enqueue(projection, target);
    }
    
    virtual void read(Matrix *out)
    {
      representation_->read(out);
    }
    
    virtual void write()
    {
      representation_->write();
      checkSynchronize();
    }
    
  protected:
    void synchronize();
    void checkSynchronize();
};

}

#endif /* GRL_TARGET_REPRESENTATION_H_ */
