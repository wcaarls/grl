/** \file cart_double_pole.cpp
 * \brief Cart-double-pole visualization source file.
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

#include <grl/visualizations/cart_double_pole.h>

using namespace grl;

REGISTER_CONFIGURABLE(CartDoublePoleVisualization) 

void CartDoublePoleVisualization::request(ConfigurationRequest *config)
{
  config->push_back(CRP("state", "signal/vector", "Cart-double-pole state to visualize", state_));
}

void CartDoublePoleVisualization::configure(Configuration &config)
{
  if (!Visualizer::instance())
    throw Exception("visualization/cart_double_pole requires a configured visualizer to run");

  state_ = (VectorSignal*)config["state"].ptr();

  // Create window  
  create("CartDoublePole");
}

void CartDoublePoleVisualization::reconfigure(const Configuration &config)
{
}

void CartDoublePoleVisualization::reshape(int width, int height)
{
  initProjection(-1.1, 1.1, -1.1, 1.1);
}

void CartDoublePoleVisualization::idle()
{
  refresh();
}

void CartDoublePoleVisualization::draw()
{
  clear();
  
  Vector state = state_->get();
  
  if (state.size())
  {
    double phi1, phi2, x;
    
    phi1 = -state[1]+M_PI/2;
    phi2 = -state[2]+M_PI/2;
    x = state[0]/2.4;
    
    drawLink(x, 0., x+0.5*cos(phi1), 0.5*sin(phi1));
    drawLink(x+0.5*cos(phi1), 0.5*sin(phi1), x+0.5*cos(phi1)+0.5*cos(phi2), 0.5*sin(phi1)+0.5*sin(phi2));
    drawJoint(x, 0.);
    drawJoint(x+0.5*cos(phi1), 0.5*sin(phi1));
    drawMass(x+0.5*cos(phi1)+0.5*cos(phi2), 0.5*sin(phi1)+0.5*sin(phi2));
  }

  swap();
}
