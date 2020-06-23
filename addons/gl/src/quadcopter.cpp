/** \file quadcopter.cpp
 * \brief Quadcopter visualization source file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2020-04-24
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

#include <grl/visualizations/pgl/quadcopter.h>

using namespace grl;

REGISTER_CONFIGURABLE(QuadcopterVisualization) 

void QuadcopterVisualization::request(ConfigurationRequest *config)
{
  PGLVisualization::request(config);
  config->push_back(CRP("limits", "vector.limits/quadcopter", "Position and velocity limits (0=unlimited)", limits_, CRP::Configuration));
}

void QuadcopterVisualization::configure(Configuration &config)
{
  PGLVisualization::configure(config);

  limits_ = config["limits"].v();
}

void QuadcopterVisualization::createScene()
{
  scene_->attach(new pgl::Sphere(0.05));
  if (limits_[0])
  {
    scene_->attach(new pgl::Box({2*limits_[0], 2*limits_[0], 0.05}, {0, 0, -1.025}))->color = {0.5, 0.5, 0.5};
    scene_->attach(new pgl::WireBox({2*limits_[0], 2*limits_[0], 2*limits_[0]}));
  }
  scene_->attach(new pgl::Arrow({-1, -1, -1}, { 0, -1, -1}, 0.02))->color = {1, 0, 0};
  scene_->attach(new pgl::Arrow({-1, -1, -1}, {-1,  0, -1}, 0.02))->color = {0, 1, 0};
  scene_->attach(new pgl::Arrow({-1, -1, -1}, {-1, -1, 0}, 0.02))->color = {0, 0, 1};
  
  scene_->attach(quadcopter_ = new pgl::Object());
  quadcopter_->attach(new pgl::Capsule({-0.3, 0, 0}, {0.3, 0, 0}, 0.02))->color = {1, 0, 0};
  quadcopter_->attach(new pgl::Capsule({0, -0.3, 0}, {0, 0.3, 0}, 0.02))->color = {0, 1, 0};

  controller_->view(0.5, 0.4, 4);
}

void QuadcopterVisualization::updateScene(const Vector &state)
{    
  quadcopter_->transform = pgl::Transform({state[6], state[7], state[8]}, {state[0], state[1], state[2]});
}
