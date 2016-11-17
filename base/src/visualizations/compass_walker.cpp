/** \file compass_walker.cpp
 * \brief Compass walker visualization source file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-03-16
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

#include <grl/visualizations/compass_walker.h>
#include <grl/environments/compass_walker/compass_walker.h>

using namespace grl;

REGISTER_CONFIGURABLE(CompassWalkerVisualization) 

void CompassWalkerVisualization::request(ConfigurationRequest *config)
{
  config->push_back(CRP("state", "signal/vector", "Compass walker state to visualize", state_));
}

void CompassWalkerVisualization::configure(Configuration &config)
{
  if (!Visualizer::instance())
    throw Exception("visualization/compass_walker requires a configured visualizer to run");

  state_ = (VectorSignal*)config["state"].ptr();

  // Create window  
  create("Compass walker");
}

void CompassWalkerVisualization::reconfigure(const Configuration &config)
{
}

void CompassWalkerVisualization::reshape(int width, int height)
{
  initProjection(-1, 10, -1, 10);
}

void CompassWalkerVisualization::idle()
{
  refresh();
}

void CompassWalkerVisualization::draw()
{
  clear();

  Vector state = state_->get();

  if (state.size())
  {
    // Make sure we can visualize observations as well
    double x = 0;
    if (state.size() > CompassWalker::siStanceFootX)
      x = state[CompassWalker::siStanceFootX];

    CSWModelState swstate;

    swstate.init(x,
                state[CompassWalker::siStanceLegAngle],
                state[CompassWalker::siStanceLegAngleRate],
                state[CompassWalker::siHipAngle],
                state[CompassWalker::siHipAngleRate]);

    drawLink(swstate.getHipX(), swstate.getHipY(), state[CompassWalker::siStanceFootX], 0);
    drawLink(swstate.getHipX(), swstate.getHipY(), swstate.getSwingFootX(), swstate.getSwingFootY());
    drawJoint(swstate.getHipX(), swstate.getHipY());
  }

  swap();
}
