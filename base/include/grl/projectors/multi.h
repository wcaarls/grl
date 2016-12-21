/** \file multi.h
 * \brief Multi projector header file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2016-12-06
 *
 * \copyright \verbatim
 * Copyright (c) 2016, Wouter Caarls
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

#ifndef GRL_MULTI_PROJECTOR_H_
#define GRL_MULTI_PROJECTOR_H_

#include <grl/projector.h>

namespace grl
{

/// Preprocesses projection onto a normalized [0, 1] vector
class MultiProjector : public Projector
{
  public:
    TYPEINFO("projector/multi", "Combines multiple projections")
    
  protected:
    int dim_;
    Vector memory_;
    std::vector<Projector*> projector_;

  public:
    MultiProjector() : dim_(-1)
    {
      projector_.resize(2, NULL);
    }

    // From Configurable
    virtual void request(const std::string &role, ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Projector
    virtual ProjectionLifetime lifetime() const { return std::max(projector_[0]->lifetime(), projector_[1]->lifetime()); }
    virtual ProjectionPtr project(const Vector &in) const;
};

}

#endif /* GRL_MULTI_PROJECTOR_H_ */
