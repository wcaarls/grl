/** \file trajectory.cpp
 * \brief Simple trajectory plot source file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-10-16
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

#include <grl/visualizations/trajectory.h>

using namespace grl;

REGISTER_CONFIGURABLE(TrajectoryVisualization) 

void TrajectoryVisualization::request(ConfigurationRequest *config)
{
  config->push_back(CRP("input_dims", "Input dimensions to visualize", dims_, CRP::Online));
  config->push_back(CRP("input_min", "Lower input dimension limit", min_, CRP::System));
  config->push_back(CRP("input_max", "Upper input dimension limit", max_, CRP::System));

  config->push_back(CRP("trajectory", "signal/matrix", "Trajectory to visualize", trajectory_));
}

void TrajectoryVisualization::configure(Configuration &config)
{
  if (!Visualizer::instance())
  {
    WARNING("visualization/trajectory requires a configured visualizer to run");
    return;
  }

  trajectory_ = (MatrixSignal*)config["trajectory"].ptr();
  timer_.restart();

  dims_ = config["input_dims"].v();
  min_ = config["input_min"].v();
  max_ = config["input_max"].v();
  if (min_.size() != max_.size())
    throw bad_param("visualization/trajectory:{input_min,input_max}");
    
  for (size_t ii=0; ii < dims_.size(); ++ii)
    if (dims_[ii] >= min_.size())
      throw bad_param("visualization/trajectory:{input_dims,input_min,input_max}");
      
  // Create window  
  create(path().c_str());
}

void TrajectoryVisualization::reconfigure(const Configuration &config)
{
  config.get("input_dims", dims_);
  for (size_t ii=0; ii < dims_.size(); ++ii)
    if (dims_[ii] >= min_.size())
      throw bad_param("visualization/trajectory:{input_dims,input_min,input_max}");
}

void TrajectoryVisualization::reshape(int width, int height)
{
  initProjection(0, 1, 0, 1);
}

void TrajectoryVisualization::idle()
{
  if (timer_.elapsed() > 0.0167)
  {
    timer_.restart();
    refresh();
  }
}

void TrajectoryVisualization::draw()
{
  glClearColor(1., 1., 1., 1.);
  clear();
  
  glColor3f(0., 0., 0.);
  glBegin(GL_LINES);
    glVertex2f(0., 0.5);
    glVertex2f(1., 0.5);
  glEnd(); 

  Matrix traj = trajectory_->get();

  if (traj.cols())
  {
    const double dx = 1./traj.cols();
      
    for (size_t dd=0; dd < dims_.size(); ++dd)
    {
      if (dims_[dd] >= traj.rows())
        throw bad_param("visualization/trajectory:input_dims");
    
      switch (dd%6)
      {
        case 0: glColor3f(0.8,0.0,0.0); break;
        case 1: glColor3f(0.0,0.8,0.0); break;
        case 2: glColor3f(0.0,0.0,0.8); break;
        case 3: glColor3f(0.0,0.8,0.8); break;
        case 4: glColor3f(0.8,0.0,0.8); break;
        case 5: glColor3f(0.8,0.8,0.0); break;
      }
    
      glBegin(GL_LINE_STRIP);
      for (size_t ii=0; ii < traj.cols(); ++ii)
        glVertex2f(ii*dx, (traj((int)dims_[dd], ii)-min_[dims_[dd]])/(max_[dims_[dd]]-min_[dims_[dd]]));
      glEnd();
    }
  }
  
  swap();
}
