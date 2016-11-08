/** \file cart_double_pole.h
 * \brief Cart-double-pole visualization header file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2016-11-08
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

#ifndef GRL_CART_DOUBLE_POLE_VISUALIZATION_H_
#define GRL_CART_DOUBLE_POLE_VISUALIZATION_H_

#include <grl/state.h>
#include <grl/visualization.h>

namespace grl
{

/// Cart-double-pole visualization.
class CartDoublePoleVisualization : public Visualization
{
  public:
    TYPEINFO("visualization/cart_double_pole", "Cart-double-pole visualization")

  protected:
    State *state_;
  
  public:
    CartDoublePoleVisualization() : state_(NULL) { }
    
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);
  
    // From Visualization
    virtual void draw();
    virtual void idle();
    virtual void reshape(int width, int height);
};

}

#endif /* GRL_CART_DOUBLE_POLE_VISUALIZATION_H_ */
