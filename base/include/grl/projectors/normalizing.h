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

/// Preprocesses projection onto a normalized [0, 1] vector
class NormalizingProjector : public Projector
{
  public:
    TYPEINFO("projector/pre/normalizing", "Preprocesses projection onto a normalized [0, 1] vector")
    
  protected:
    Projector *projector_;
    
    Vector min_, max_, scaling_;

  public:
    NormalizingProjector() : projector_(NULL) { }

    // From Configurable
    virtual void request(const std::string &role, ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Projector
    virtual ProjectionLifetime lifetime() const { return projector_->lifetime(); }
    virtual ProjectionPtr project(const Vector &in) const;
    virtual Matrix jacobian(const Vector &in) const;
};

}

#endif /* GRL_NORMALIZING_PROJECTOR_H_ */
