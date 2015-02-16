/** \file pendulum.cpp
 * \brief Pendulum visualization source file.
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

#include <GL/gl.h>
#include <GL/glu.h>

#include <grl/visualizations/pendulum.h>

#define EPS 0.001

using namespace grl;

REGISTER_CONFIGURABLE(PendulumVisualization) 

void PendulumVisualization::request(ConfigurationRequest *config)
{
  config->push_back(CRP("state", "state", "Pendulum state to visualize", state_));
}

void PendulumVisualization::configure(Configuration &config)
{
  if (!Visualizer::instance())
    throw Exception("visualization/pendulum requires a configured visualizer to run");

  state_ = (State*)config["state"].ptr();

  // Create window  
  create("Pendulum");
}

void PendulumVisualization::reconfigure(const Configuration &config)
{
}

void PendulumVisualization::reshape(int width, int height)
{
  initProjection(-1.1, 1.1, -1.1, 1.1);
}

void PendulumVisualization::idle()
{
  refresh();
}

void PendulumVisualization::draw()
{
  clear();
  
  Vector state = state_->get();
  
  if (!state.empty())
  {
    double phi = -state[0]+M_PI/2;
  
    drawLink(0, 0, cos(phi), sin(phi));
    drawJoint(0, 0);
    drawMass(cos(phi), sin(phi));
  }

  swap();
}
