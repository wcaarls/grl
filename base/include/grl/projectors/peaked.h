/** \file peaked.h
 * \brief Peaked projector header file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-09-01
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

#ifndef GRL_PEAKED_PROJECTOR_H_
#define GRL_PEAKED_PROJECTOR_H_

#include <grl/projectors/normalizing.h>

namespace grl
{

/// Preprocesses projection for more resolution around center
class PeakedProjector : public NormalizingProjector
{
  public:
    TYPEINFO("projector/pre/peaked", "Preprocesses projection for more resolution around center")
    
  protected:
    Vector peaking_, range2_;

  public:
    // From Configurable
    virtual void request(const std::string &role, ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Projector
    virtual ProjectionPtr project(const Vector &in) const;
    virtual Matrix jacobian(const Vector &in) const;
};

}

#endif /* GRL_PEAKED_PROJECTOR_H_ */
