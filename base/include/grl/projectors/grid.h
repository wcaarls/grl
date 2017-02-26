/** \file grid.h
 * \brief Grid projector header file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-08-28
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

#ifndef GRL_GRID_PROJECTOR_H_
#define GRL_GRID_PROJECTOR_H_

#include <grl/projector.h>
#include <grl/discretizer.h>

namespace grl
{

/// Projects onto a grid
class GridProjector : public Projector
{
  protected:
    Discretizer* discretizer_;
    
  public:
    // From Configurable
    virtual void request(const std::string &role, ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Projector
    virtual ProjectionLifetime lifetime() const { return plIndefinite; }
};

/// Project to linear grid index.
class GridIndexProjector : public GridProjector
{
  public:
    TYPEINFO("projector/grid/index", "Discretizes continuous input to a linear grid index")
    
  public:
    // From Projector
    virtual ProjectionPtr project(const Vector &in) const;
};

/// Project to discretized position.
class GridPositionProjector : public GridProjector
{
  public:
    TYPEINFO("projector/grid/position", "Discretizes continuous input to a grid center position")
    
  public:
    // From Projector
    virtual ProjectionPtr project(const Vector &in) const;
};

}

#endif /* GRL_GRID_PROJECTOR_H_ */
