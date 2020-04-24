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

#include <grl/visualizations/quadcopter.h>

using namespace grl;

REGISTER_CONFIGURABLE(QuadcopterVisualization) 

void QuadcopterVisualization::request(ConfigurationRequest *config)
{
  config->push_back(CRP("limits", "vector.limits/quadcopter", "Position and velocity limits (0=unlimited)", limits_, CRP::Configuration));
  config->push_back(CRP("state", "signal/vector", "Quadcopter state to visualize", state_));
}

void QuadcopterVisualization::configure(Configuration &config)
{
  if (!Visualizer::instance())
    throw Exception("visualization/quadcopter requires a configured visualizer to run");

  limits_ = config["limits"].v();
  state_ = (VectorSignal*)config["state"].ptr();
  
  // Create window  
  create("Quadcopter");
  
}

void QuadcopterVisualization::reconfigure(const Configuration &config)
{
}

void QuadcopterVisualization::reshape(int width, int height)
{
  glViewport(0, 0, width, height);
}

void QuadcopterVisualization::idle()
{
  refresh();
}

void QuadcopterVisualization::draw()
{
  if (!scene_)
  {
    // Create scene
    scene_ = new pgl::Scene();
    scene_->attach(new pgl::Sphere(0.05));
    scene_->attach(new pgl::Arrow({-1, -1, -1}, { 0, -1, -1}, 0.02))->color = {1, 0, 0};
    scene_->attach(new pgl::Arrow({-1, -1, -1}, {-1,  0, -1}, 0.02))->color = {0, 1, 0};
    scene_->attach(new pgl::Arrow({-1, -1, -1}, {-1, -1, 0}, 0.02))->color = {0, 0, 1};
    if (limits_[0])
    {
      scene_->attach(new pgl::Box({2*limits_[0], 2*limits_[0], 0.05}, {0, 0, -1.025}));
      scene_->attach(new pgl::WireBox({2*limits_[0], 2*limits_[0], 2*limits_[0]}));
    }
    
    scene_->attach(quadcopter_ = new pgl::Capsule({-0.3, 0, 0}, {0.3, 0, 0}, 0.02))->color = {1, 0, 0};
    quadcopter_->attach(new pgl::Capsule({0, -0.3, 0}, {0, 0.3, 0}, 0.02))->color = {0, 1, 0};
    
    controller_ = new pgl::OrbitController(scene_);
    controller_->view(0.5, 0.4, 4);
    
    // Setup OpenGL
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_COLOR_MATERIAL);
    glEnable(GL_CULL_FACE);
    glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
    glCullFace(GL_BACK);
  }

  Vector state = state_->get();
  pgl::Transform old = quadcopter_->transform;
  
  if (state.size())
    quadcopter_->transform = pgl::Transform({state[6], state[7], state[8]}, {state[0], state[1], state[2]}) * quadcopter_->transform;
  
  scene_->draw();
  quadcopter_->transform = old;

  swap();
}
