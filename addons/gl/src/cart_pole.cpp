/** \file cart_pole.cpp
 * \brief PGL-based cart-pole visualization source file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2020-12-09
 *
 * \copyright \verbatim
 * Copyright (c) 2020, Wouter Caarls
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

#include <grl/visualizations/pgl/cart_pole.h>

using namespace grl;

REGISTER_CONFIGURABLE(PGLCartPoleVisualization) 

void PGLCartPoleVisualization::createScene()
{
  cart_ = scene_->attach(new pgl::Box({0.2, 0.1, 0.1}));
  pole_ = cart_->attach(new pgl::Cylinder(0.5, 0.01));
  pole_->color = {0, 0, 1};
  
  scene_->attach(new pgl::Box({2.4, 0.1, 0.01}, {0, 0, -0.055}))->color = {1, 0, 0};
  scene_->attach(new pgl::Box({0.01, 0.2, 0.011}, {0, 0, -0.055}))->color = {0, 1, 0};
  
  controller_->view(0.5, 0.5, 3);
}

void PGLCartPoleVisualization::updateScene(const Vector &state)
{    
  cart_->transform = pgl::Translation({state[0], 0, 0});
  pole_->transform = pgl::Transform({0, -1, 0}, state[0], {0, -0.06, 0.25});
}
