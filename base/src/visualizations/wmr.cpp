/** \file wmr.cpp
 * \brief Wheeled mobile robot visualization source file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2023-03-13
 *
 * \copyright \verbatim
 * Copyright (c) 2023, Wouter Caarls
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

#include <grl/visualizations/wmr.h>

using namespace grl;

REGISTER_CONFIGURABLE(WMRVisualization) 

void WMRVisualization::request(ConfigurationRequest *config)
{
  config->push_back(CRP("state", "signal/vector", "Wheeled mobile robot state to visualize", state_));
}

void WMRVisualization::configure(Configuration &config)
{
  if (!Visualizer::instance())
    throw Exception("visualization/wmr requires a configured visualizer to run");

  state_ = (VectorSignal*)config["state"].ptr();
  
  t_ = 1.0; // Track (horizontal size)
  l_ = 0.2; // Caster wheel support length
  b_ = 1.0; // Wheelbase (front to back distance)

  // Create window  
  create("Wheeled mobile robot");
}

void WMRVisualization::reconfigure(const Configuration &config)
{
}

void WMRVisualization::reshape(int width, int height)
{
  initProjection(-10.1, 10.1, -10.1, 10.1);
}

void WMRVisualization::idle()
{
  refresh();
}

Eigen::Matrix3d R(double angle)
{
  Eigen::Matrix3d M;
  
  M << cos(angle), -sin(angle), 0,
       sin(angle),  cos(angle), 0,
       0,           0,          1;
       
  return M;
}

Eigen::Matrix3d T(double x, double y)
{
  Eigen::Matrix3d M;
  
  M << 1, 0, x, 0, 1, y, 0, 0, 1;
  
  return M;
}

void WMRVisualization::draw()
{
  clear();
  
  Vector state = state_->get();
  
  if (state.size())
  {
    Eigen::Matrix3d bl = T(state[0], state[1])*R(state[2]);
    Eigen::Vector3d leftwheel  = bl*Eigen::Vector3d(0,  t_/2, 1),
                    rightwheel = bl*Eigen::Vector3d(0, -t_/2, 1);
                    
    drawLink(leftwheel[0], leftwheel[1], rightwheel[0], rightwheel[1]);
    drawMass(state[0], state[1]);
    
    if (state.size() >= 5)
    {
      Eigen::Matrix3d lc = bl * T(-b_,  t_/2) * R(state[3]),
                      rc = bl * T(-b_, -t_/2) * R(state[4]);
      Eigen::Vector3d lcmount = lc*Eigen::Vector3d(0, 0, 1),
                      rcmount = rc*Eigen::Vector3d(0, 0, 1),
                      lcwheel = lc*Eigen::Vector3d(-l_, 0, 1),
                      rcwheel = rc*Eigen::Vector3d(-l_, 0, 1);
               
      drawLink(lcmount[0], lcmount[1], lcwheel[0], lcwheel[1]);
      drawLink(rcmount[0], rcmount[1], rcwheel[0], rcwheel[1]);
      drawMass(lcmount[0], lcmount[1]);
      drawMass(rcmount[0], rcmount[1]);
    }
  }

  swap();
}
