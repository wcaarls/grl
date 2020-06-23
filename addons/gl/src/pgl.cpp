/** \file quadcopter.cpp
 * \brief PGL visualization source file.
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

#include <grl/visualizations/pgl.h>

using namespace grl;

void PGLVisualization::request(ConfigurationRequest *config)
{
  config->push_back(CRP("state", "signal/vector", "State to visualize", state_));
}

void PGLVisualization::configure(Configuration &config)
{
  if (!Visualizer::instance())
  {
    WARNING("visualization/pgl requires a configured visualizer to run");
    return;
  }
 
  state_ = (VectorSignal*)config["state"].ptr();
  
  // Create window  
  create(path().c_str());
}

void PGLVisualization::reconfigure(const Configuration &config)
{
}

void PGLVisualization::reshape(int width, int height)
{
  glViewport(0, 0, width, height);
}

void PGLVisualization::click(int button, int state, int x, int y)
{
  if (controller_)
  {
    // Convert to glfw
    if (button < 3)
      controller_->click(button==0?0:3-button, !state, 0, (double)x, (double)y);
    else if (button < 5 && state == 1)
      controller_->scroll(0, 2*(button == 3)-1);
  }
}

void PGLVisualization::motion(int x, int y)
{
  if (controller_)
    controller_->motion((double)x, (double)y);
}

void PGLVisualization::idle()
{
  refresh();
}

void PGLVisualization::draw()
{
  if (!scene_)
  {
    // Setup OpenGL
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_COLOR_MATERIAL);
    glEnable(GL_CULL_FACE);
    glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
    glCullFace(GL_BACK);
    
    // Create scene
    scene_ = new pgl::Scene();
    camera_ = new pgl::Camera(scene_);
    controller_ = new pgl::OrbitController(camera_);
    
    // Make subclass populate scene
    createScene();
  }
  
  Vector state = state_->get();
  if (state.size())
    updateScene(state);

  camera_->draw();
  swap();
}
