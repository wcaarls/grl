/** \file flyer2d.cpp
 * \brief 2D flyer visualization source file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2016-07-13
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

#include <grl/visualizations/flyer2d.h>

using namespace grl;

REGISTER_CONFIGURABLE(Flyer2DVisualization) 

void Flyer2DVisualization::request(ConfigurationRequest *config)
{
  config->push_back(CRP("state", "signal/vector", "2D flyer state to visualize", state_));
}

void Flyer2DVisualization::configure(Configuration &config)
{
  if (!Visualizer::instance())
    throw Exception("visualization/flyer2d requires a configured visualizer to run");

  state_ = (VectorSignal*)config["state"].ptr();

  // Create window  
  create("2D flyer");
}

void Flyer2DVisualization::reconfigure(const Configuration &config)
{
}

void Flyer2DVisualization::reshape(int width, int height)
{
  initProjection(-1.1, 1.1, -1.1, 1.1);
}

void Flyer2DVisualization::idle()
{
  refresh();
}

void Flyer2DVisualization::draw()
{
  clear();
  
  Vector state = state_->get();
  
  if (state.size())
  {
    double phi = state[2];
  
    drawLink(state[0]-0.1*cos(phi), state[1]-0.1*sin(phi), state[0]+0.1*cos(phi), state[1]+0.1*sin(phi));
    drawMass(state[0], state[1]);
  }

  swap();
}
