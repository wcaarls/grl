/** \file cart_pole.h
 * \brief Cart-pole visualization header file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-02-15
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

#ifndef GRL_CART_POLE_VISUALIZATION_H_
#define GRL_CART_POLE_VISUALIZATION_H_

#include <grl/state.h>
#include <grl/visualization.h>

namespace grl
{

/// Cart-pole visualization.
class CartPoleVisualization : public Visualization
{
  public:
    TYPEINFO("visualization/cart_pole")

  protected:
    State *state_;
    std::string variant_;
  
  public:
    CartPoleVisualization() : state_(NULL), variant_("mprl") { }
    
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

#endif /* GRL_CART_POLE_VISUALIZATION_H_ */
