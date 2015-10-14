/** \file acrobot.cpp
 * \brief Acrobot visualization source file.
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

#include <grl/visualizations/acrobot.h>

using namespace grl;

REGISTER_CONFIGURABLE(AcrobotVisualization) 

void AcrobotVisualization::request(ConfigurationRequest *config)
{
  config->push_back(CRP("state", "state", "Acrobot state to visualize", state_));
}

void AcrobotVisualization::configure(Configuration &config)
{
  if (!Visualizer::instance())
    throw Exception("visualization/acrobot requires a configured visualizer to run");

  state_ = (State*)config["state"].ptr();

  // Create window  
  create("Acrobot");
}

void AcrobotVisualization::reconfigure(const Configuration &config)
{
}

void AcrobotVisualization::reshape(int width, int height)
{
  initProjection(-2.1, 2.1, -2.1, 2.1);
}

void AcrobotVisualization::idle()
{
  refresh();
}

void AcrobotVisualization::draw()
{
  clear();
  
  Vector state = state_->get();
  
  if (state.size())
  {
    double theta1 = state[0] - M_PI/2,
           theta2 = state[0] + state[1] - M_PI/2;
  
    drawLink(0, 0, cos(theta1), sin(theta1));
    drawLink(cos(theta1), sin(theta1), cos(theta1)+cos(theta2), sin(theta1)+sin(theta2));
    drawJoint(0, 0);
    drawJoint(cos(theta1), sin(theta1));
    drawMass(0.5*cos(theta1), 0.5*sin(theta1));
    drawMass(cos(theta1)+0.5*cos(theta2), sin(theta1)+0.5*sin(theta2));
  }

  swap();
}
