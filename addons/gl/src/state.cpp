/** \file state.cpp
 * \brief Simple state plot source file.
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

#include <grl/visualizations/state.h>

#define EPS 0.001

using namespace grl;

REGISTER_CONFIGURABLE(StateVisualization) 

void StateVisualization::request(ConfigurationRequest *config)
{
  config->push_back(CRP("input_dims", "Input dimensions to visualize", dims_, CRP::Online));
  config->push_back(CRP("input_min", "Lower input dimension limit", min_, CRP::System));
  config->push_back(CRP("input_max", "Upper input dimension limit", max_, CRP::System));
  config->push_back(CRP("memory", "Number of data points to draw", (int)memory_, CRP::Online));

  config->push_back(CRP("state", "signal/vector", "State to visualize", state_));
  config->push_back(CRP("exporter", "exporter", "Optional exporter for logging (supports value)", exporter_, true));
}

void StateVisualization::configure(Configuration &config)
{
  if (!Visualizer::instance())
  {
    WARNING("visualization/state requires a configured visualizer to run");
    return;
  }

  state_ = (VectorSignal*)config["state"].ptr();
  state_reader_ = state_->addReader();
  
  exporter_ = (Exporter*) config["exporter"].ptr();
  if (exporter_)
  {
    exporter_->init({"value"});
    exporter_->open("", false);
    exporter_->open();
  }

  dims_ = config["input_dims"].v();
  min_ = config["input_min"].v();
  max_ = config["input_max"].v();
  if (min_.size() != max_.size())
    throw bad_param("visualization/state:{input_min,input_max}");
    
  if (!dims_.size())
  {
    dims_ = Vector(min_.size());
    for (size_t ii=0; ii < min_.size(); ++ii)
      dims_[ii] = ii;
  }
  
  if (min_.size() == 1)
  {
    size_t maxdim = dims_.maxCoeff()+1;
  
    min_ = ConstantVector(maxdim, min_[0]);
    max_ = ConstantVector(maxdim, max_[0]);
  }
  
  for (size_t ii=0; ii < dims_.size(); ++ii)
    if (dims_[ii] >= min_.size())
      throw bad_param("visualization/state:{input_dims,input_min,input_max}");
      
  memory_ = config["memory"];

  // Create window  
  create(path().c_str());

  // Let's get this show on the road
  start();
}

void StateVisualization::reconfigure(const Configuration &config)
{
  config.get("input_dims", dims_);
  for (size_t ii=0; ii < dims_.size(); ++ii)
    if (dims_[ii] >= min_.size())
      throw bad_param("visualization/state:{input_dims,input_min,input_max}");

  config.get("memory", memory_);
}

void StateVisualization::reshape(int width, int height)
{
  initProjection(0, 1, 0, 1);
}

void StateVisualization::run()
{
  while (ok())
  {
    Vector state = state_reader_.read(), point(dims_.size());
    
    // Handle wake up, in which case no valid data is returned
    if (!state_reader_.engaged())
      continue;
    
    if (exporter_)
      exporter_->write({state});
    
    for (size_t ii=0; ii < dims_.size(); ++ii)
    {
      size_t d = dims_[ii];
      double norm = fmin(fmax((state[d]-min_[d])/(max_[d]-min_[d]), 0.), 1.);
      
      point[ii] = norm*0.98+0.01;
    }
      
    Guard guard(mutex_);
    points_.push_back(point);
    while (points_.size() > memory_)
      points_.pop_front();
    
    updated_ = true;
  }
}

void StateVisualization::idle()
{
  if (updated_)
    refresh();
}

void StateVisualization::draw()
{
  const double dx = 1./memory_;

  if (updated_)
  {
    if (list_)
      glDeleteLists(list_, 1);
    
    list_ = glGenLists(1);
    
    glNewList(list_, GL_COMPILE);

    Guard guard(mutex_);
    for (size_t dd=0; dd < dims_.size(); ++dd)
    {
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
      double xx=0;
      for (std::deque<Vector>::iterator it=points_.begin(); it != points_.end(); ++it, xx+=dx)
        glVertex2f(xx, (*it)[dd]);
      glEnd();
    }
    glEndList();
    
    updated_ = false;
  }

  glClearColor(1., 1., 1., 1.);
  clear();
  
  glColor3f(0., 0., 0.);
  glBegin(GL_LINES);
    glVertex2f(0., 0.5);
    glVertex2f(1., 0.5);
  glEnd(); 

  glCallList(list_);

  swap();
}
