/** \file cart_pole.cpp
 * \brief CartPole visualization source file.
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

#include <grl/visualizations/cart_pole.h>

#define EPS 0.001

using namespace grl;

REGISTER_CONFIGURABLE(CartPoleVisualization) 

void CartPoleVisualization::request(ConfigurationRequest *config)
{
  config->push_back(CRP("state", "state", "Cart-pole state to visualize", state_));
}

void CartPoleVisualization::configure(Configuration &config)
{
  if (!Visualizer::instance())
    throw Exception("visualization/cart_pole requires a configured visualizer to run");

  state_ = (State*)config["state"].ptr();

  // Create window  
  create("CartPole");
}

void CartPoleVisualization::reconfigure(const Configuration &config)
{
}

void CartPoleVisualization::reshape(int width, int height)
{
  initProjection(-1.1, 1.1, -1.1, 1.1);
}

void CartPoleVisualization::idle()
{
  refresh();
}

void CartPoleVisualization::draw()
{
  clear();
  
  Vector state = state_->get();
  
  if (!state.empty())
  {
    double phi = -state[2]+M_PI/2;
    double x = state[0]/2.4;
  
    drawLink(x, 0., x+cos(phi), sin(phi));
    drawJoint(x, 0.);
    drawMass(x+cos(phi), sin(phi));
  }

  swap();
}
