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
  config->push_back(CRP("track", "Vehicle track (horizontal size)", t_));
  config->push_back(CRP("base", "Wheel base (front to back distance)", b_));
  config->push_back(CRP("length", "Caster wheel support length", l_));
  config->push_back(CRP("sensor_pos", "Position of sensor bar w.r.t wheels", sensor_pos_, CRP::Configuration));
  config->push_back(CRP("sensor_width", "Width of sensor bar", sensor_width_, CRP::Configuration));

  config->push_back(CRP("state", "signal/vector", "Wheeled mobile robot state to visualize", state_));
  config->push_back(CRP("trajectory", "mapping/image", "Image containing trajectory to follow", trajectory_, true));
}

void WMRVisualization::configure(Configuration &config)
{
  if (!Visualizer::instance())
    throw Exception("visualization/wmr requires a configured visualizer to run");

  t_ = config["track"];
  b_ = config["base"];
  l_ = config["length"];
  sensor_pos_ = config["sensor_pos"];
  sensor_width_ = config["sensor_width"];
  
  state_ = (VectorSignal*)config["state"].ptr();
  trajectory_ = (Mapping*)config["trajectory"].ptr();
  
  if (trajectory_)
  {
    min_ = (*this)["trajectory/min"].v(),
    max_ = (*this)["trajectory/max"].v();
    
    data_ = new unsigned char[1000000];
    for (size_t yy=0; yy != 1000; ++yy)
      for (size_t xx=0; xx != 1000; ++xx)
      {
        Vector res;
        data_[yy*1000+xx] = 255*trajectory_->read(min_ + VectorConstructor(xx/1000., yy/1000.)*(max_-min_), &res);
      }
  }
  else
  {
    min_ = VectorConstructor(-10.1, -10.1);
    max_ = VectorConstructor( 10.1,  10.1);
  }

  // Create window  
  create("Wheeled mobile robot");
}

void WMRVisualization::reconfigure(const Configuration &config)
{
}

void WMRVisualization::reshape(int width, int height)
{
  initProjection(min_[0], max_[0], min_[1], max_[1]);
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

    if (trajectory_)
    {
      drawTexture(0, 0, 1, 1, data_, 1000, 1000, false);
      
      Eigen::Vector3d leftsensor  = bl*Eigen::Vector3d(sensor_pos_,  sensor_width_/2, 1),
                      rightsensor = bl*Eigen::Vector3d(sensor_pos_, -sensor_width_/2, 1);

      drawLink(leftsensor[0], leftsensor[1], rightsensor[0], rightsensor[1]);
    }
  
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
