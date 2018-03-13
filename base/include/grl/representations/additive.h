/** \file additive.h
 * \brief Additive representation header file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-11-15
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

#ifndef GRL_ADDITIVE_REPRESENTATION_H_
#define GRL_ADDITIVE_REPRESENTATION_H_

#include <grl/representation.h>

namespace grl
{

/// Linear combination of two representations
class AdditiveRepresentation : public Representation
{
  public:
    TYPEINFO("representation/additive", "Linear combination of two representations")

  protected:
    TypedConfigurableList<Representation> representation_;
    int learning_;
    
  public:
    AdditiveRepresentation() : learning_(0)
    {
    }
    
    // From Configurable
    virtual void request(const std::string &role, ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);
  
    // From Representation
    virtual double read(const ProjectionPtr &projection, Vector *result, Vector *stddev) const;
    virtual void write(const ProjectionPtr projection, const Vector &target, const Vector &alpha);
    virtual void update(const ProjectionPtr projection, const Vector &delta);
};

}

#endif /* GRL_ADDITIVE_REPRESENTATION_H_ */
