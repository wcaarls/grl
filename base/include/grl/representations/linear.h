/** \file linear.h
 * \brief Linear representation header file.
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
/*
 * linear_representation.h
 *
 *  Created on: Jul 30, 2014
 *      Author: wcaarls
 */

#ifndef GRL_LINEAR_REPRESENTATION_H_
#define GRL_LINEAR_REPRESENTATION_H_

#include <grl/representation.h>

namespace grl
{

/// Average of feature activations.
class LinearRepresentation : public ParameterizedRepresentation
{
  public:
    TYPEINFO("representation/parameterized/linear")
    
  protected:
    Vector min_, max_, params_;
    size_t memory_, outputs_;

  public:
    LinearRepresentation() : memory_(8*1024*1024), outputs_(1)
    {
      min_ = VectorConstructor(0.);
      max_ = VectorConstructor(1.);
    }
  
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);
  
    // From ParameterizedRepresentation
    virtual LinearRepresentation *clone() const;
    virtual double read(const ProjectionPtr &projection, Vector *result) const ;
    virtual void write(const ProjectionPtr projection, const Vector &target, double alpha=1);
    virtual void update(const ProjectionPtr projection, const Vector &delta);
    
    virtual size_t size() const
    {
      return params_.size()/outputs_;
    }
    
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

#endif /* GRL_LINEAR_REPRESENTATION_H_ */
