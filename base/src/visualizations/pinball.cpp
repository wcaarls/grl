/** \file pinball.cpp
 * \brief Pinball visualization source file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-08-13
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

#include <grl/visualizations/pinball.h>

using namespace grl;

REGISTER_CONFIGURABLE(PinballVisualization) 

void PinballVisualization::request(ConfigurationRequest *config)
{
  config->push_back(CRP("state", "signal/vector", "Pinball state to visualize", state_));
}

void PinballVisualization::configure(Configuration &config)
{
  if (!Visualizer::instance())
    throw Exception("visualization/pinball requires a configured visualizer to run");

  state_ = (VectorSignal*)config["state"].ptr();

  // Create window  
  create("Pinball");
}

void PinballVisualization::reconfigure(const Configuration &config)
{
}

void PinballVisualization::reshape(int width, int height)
{
  initProjection(-0.1, 1.1, -0.1, 1.1);
}

void PinballVisualization::idle()
{
  refresh();
}

void PinballVisualization::draw()
{
  clear();
  
  Vector state = state_->get();
  
  drawSurface( 0,  0,  1,  1, .1, .1, .5);
  drawSurface(.2,  0, .4, .8,  0,  0,  0);
  drawSurface(.6, .2, .8,  1,  0,  0,  0);
  
  if (state.size() > 2)
    drawMass(state[0], state[1]);

  swap();
}
