/** \file windy.cpp
 * \brief Windy gridworld visualization source file.
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

#include <grl/visualizations/windy.h>

using namespace grl;

REGISTER_CONFIGURABLE(WindyGridworldVisualization) 

void WindyGridworldVisualization::request(ConfigurationRequest *config)
{
  config->push_back(CRP("state", "state", "Windy gridworld state to visualize", state_));
}

void WindyGridworldVisualization::configure(Configuration &config)
{
  if (!Visualizer::instance())
    throw Exception("visualization/windy requires a configured visualizer to run");

  state_ = (State*)config["state"].ptr();

  // Create window  
  create("Windy gridworld");
}

void WindyGridworldVisualization::reconfigure(const Configuration &config)
{
}

void WindyGridworldVisualization::reshape(int width, int height)
{
  initProjection(0, 10, 0, 7);
}

void WindyGridworldVisualization::idle()
{
  refresh();
}

void WindyGridworldVisualization::draw()
{
  clear();
  
  Vector state = state_->get();

  for (size_t xx=0; xx < 10; ++xx)
    for (size_t yy=0; yy < 7; ++yy)
      drawSurface(xx+0.1, yy+0.1, xx+0.9, yy+0.9,
                  0.1+(xx==0&&yy==3)*0.4,
                  0.1+(xx==7&&yy==3)*0.4,
                  0.5+(xx>2&&xx<9)*0.2+(xx>5&&xx<8)*0.3);
  
  if (state.size() > 2)
    drawMass(state[0]+0.5, state[1]+0.5);

  swap();
}
