/** \file normalizing.h
 * \brief Normalizing projector header file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-03-10
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

#ifndef GRL_NORMALIZING_PROJECTOR_H_
#define GRL_NORMALIZING_PROJECTOR_H_

#include <grl/projector.h>

namespace grl
{

/// Projects the input onto a normalized vector
class NormalizingProjector : public Projector
{
  public:
    TYPEINFO("projector/normalizing", "Projects the input onto a normalized vector")
    
  protected:
    Vector min_, max_, scaling_;
    size_t dims_;

  public:
    NormalizingProjector() : dims_(0) { }

    // From Configurable
    virtual void request(const std::string &role, ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Projector
    virtual NormalizingProjector *clone() const;
    virtual ProjectionPtr project(const Vector &in) const;
};

}

#endif /* GRL_NORMALIZING_PROJECTOR_H_ */
