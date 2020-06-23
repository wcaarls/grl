/** \file pendulum.cpp
 * \brief PGL-based pendulum visualization source file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2020-06-23
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

#include <grl/visualizations/pgl/pendulum.h>

using namespace grl;

REGISTER_CONFIGURABLE(PGLPendulumVisualization) 

void PGLPendulumVisualization::createScene()
{
  auto casing = scene_->attach(new pgl::Box({0.15, 0.15, 0.15}, {0, 0.075, 0}));
  casing->color = {0, 0, 0.8};
  pendulum_ = casing->attach(new pgl::Object());
  
  auto disk = pendulum_->attach(new pgl::Cylinder({0, -0.08, 0}, {0, -0.081, 0}, 0.06));
  disk->attach(new pgl::Cylinder({0, 0.042, 0}, {0, 0.042, 0.01}, 0.02))->color = {0.5, 0.5, 0.5};
  
  controller_->view(0.5, 0.5, 0.25);
}

void PGLPendulumVisualization::updateScene(const Vector &state)
{    
  pendulum_->transform = pgl::Transform({0, -1, 0}, state[0], {0, 0, 0});
}
