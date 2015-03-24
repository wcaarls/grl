/** \file state.cpp
 * \brief Simple state visualization source file.
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
  config->push_back(CRP("field_dims", "Dimensions to visualize", dims_));
  config->push_back(CRP("field_min", "Lower visualization dimension limit", min_, CRP::System));
  config->push_back(CRP("field_max", "Upper visualization dimension limit", max_, CRP::System));

  config->push_back(CRP("state", "state", "State to visualize", state_));
}

void StateVisualization::configure(Configuration &config)
{
  if (!Visualizer::instance())
    throw Exception("visualization/state requires a configured visualizer to run");

  state_ = (State*)config["state"].ptr();

  dims_ = config["field_dims"];
  if (dims_.size() != 2)
    throw bad_param("visualization/state:field_dims");
  min_ = config["field_min"];
  if (min_.size() != 2)
    throw bad_param("visualization/state:field_min");
  max_ = config["field_max"];
  if (max_.size() != 2)
    throw bad_param("visualization/state:field_max");

  // Create window  
  create("State");
}

void StateVisualization::reconfigure(const Configuration &config)
{
}

void StateVisualization::reshape(int width, int height)
{
  initProjection(-1, 1, -1, 1);
}

void StateVisualization::idle()
{
  refresh();
}

void StateVisualization::draw()
{
  clear();

  const Vector state = state_->get();
  
  if (state.size())
  {
    Vector scaled = (VectorConstructor(state[dims_[0]], state[dims_[1]])-min_)/(max_-min_)*2-1;
    
    glBegin(GL_LINES);
      glVertex2d(scaled[0]-0.05, scaled[1]);
      glVertex2d(scaled[0]+0.05, scaled[1]);
      glVertex2d(scaled[0], scaled[1]-0.05);
      glVertex2d(scaled[0], scaled[1]+0.05);
    glEnd();
  }

  swap();
}
