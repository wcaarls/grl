/** \file sample.cpp
 * \brief Sample visualization source file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-01-26
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

#include <grl/visualizations/sample.h>

#define EPS 0.001

using namespace grl;

REGISTER_CONFIGURABLE(SampleVisualization) 

void SampleVisualization::request(ConfigurationRequest *config)
{
  config->push_back(CRP("field_dims", "Dimensions to visualize", dims_));
  config->push_back(CRP("field_min", "Lower visualization dimension limit", min_, CRP::System));
  config->push_back(CRP("field_max", "Upper visualization dimension limit", max_, CRP::System));
  config->push_back(CRP("output_dim", "Output dimension to visualize", dim_));
  config->push_back(CRP("points", "Texture size", points_));

  config->push_back(CRP("projector", "projector/sample", "Sample projector whose store to visualize", projector_));
}

void SampleVisualization::configure(Configuration &config)
{
  if (!Visualizer::instance())
    throw Exception("visualization/sample requires a configured visualizer to run");

  projector_ = (SampleProjector*)config["projector"].ptr();

  dims_ = config["field_dims"];
  if (dims_.size() != 2)
    throw bad_param("visualization/sample:field_dims");
  min_ = config["field_min"];
  if (min_.size() != 2)
    throw bad_param("visualization/sample:field_min");
  max_ = config["field_max"];
  if (max_.size() != 2)
    throw bad_param("visualization/sample:field_max");
  dim_ = config["output_dim"];  
  points_ = config["points"];
  
  // Divide points among dimensions
  dimpoints_ = pow(points_, 0.5);
  points_ = pow(dimpoints_, 2);
  
  // Allocate texture
  data_ = (unsigned char*) malloc(points_*3*sizeof(unsigned char));
  
  // Create window  
  create("Samples");
}

void SampleVisualization::reconfigure(const Configuration &config)
{
}

void SampleVisualization::reshape(int width, int height)
{
  initProjection(-1, 1, -1, 1);
}

// TODO: split off into new thread
void SampleVisualization::idle()
{
  float *value = new float[points_];
  memset(value, 0xFF, points_*sizeof(float));
  int d0 = (int)dims_[0], d1 = (int)dims_[1];

  float value_max=-std::numeric_limits<float>::infinity(),
        value_min= std::numeric_limits<float>::infinity();

  // Gather data
  StorePtr store = projector_->store();
  
  if (store)
  {
    Guard guard(*store); 
    
    for (size_t ii=0; ii < store->size(); ++ii)
    {
      Sample *s = (*store)[ii];
      
      if (s->in[d0] < min_[0] || s->in[d0] > max_[0] ||
          s->in[d1] < min_[1] || s->in[d1] > max_[1])
        continue;
      
      int dx = std::max(std::min((int)(dimpoints_*(s->in[d0]-min_[0])/(max_[0]-min_[0])), dimpoints_-1), 0),
          dy = std::max(std::min((int)(dimpoints_*(s->in[d1]-min_[1])/(max_[1]-min_[1])), dimpoints_-1), 0);
          
      double v = s->out[dim_];
      
      value[dy*dimpoints_+dx] = v;
      value_max = fmax(v, value_max);
      value_min = fmin(v, value_min);
    }
  }
  
//  value_min = -2000;
//  value_max = 0;
  
  float value_range = value_max-value_min;
  
  // Create texture
  for (int ii=0; ii < points_; ++ii)
  {
    double v = (value[ii] - value_min)/value_range;
    double v2 = 4*v;
    
    // Jet colormap
    if (!isnan(value[ii]))
    {
      data_[ii*3+0] = fmax(fmin(255*fmin(v2 - 1.5, -v2 + 4.5), 255), 0);
      data_[ii*3+1] = fmax(fmin(255*fmin(v2 - 0.5, -v2 + 3.5), 255), 0);
      data_[ii*3+2] = fmax(fmin(255*fmin(v2 + 0.5, -v2 + 2.5), 255), 0);
    }
    else
      data_[ii*3+0] = data_[ii*3+1] = data_[ii*3+2] = 0;
  }
  
  delete[] value;
  
  value_min_ = value_min;
  value_max_ = value_max;
  
//  std::cout << value_min << " - " << value_max << std::endl;

  // Redisplay  
  updated_ = true;
  refresh();
}

void SampleVisualization::draw()
{
  if (updated_)
  {
    if (!texture_)
      glGenTextures(1, &texture_);
  
    // Set texture
    glBindTexture( GL_TEXTURE_2D, texture_ );
    glTexEnvf( GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE );
    glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_NEAREST );
    glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
    glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE );
    glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE );
    
    gluBuild2DMipmaps( GL_TEXTURE_2D, 3, dimpoints_, dimpoints_, GL_RGB, GL_UNSIGNED_BYTE, data_ );
    
    updated_ = false;
  }

  clear();
  
  // Draw texture
  glEnable(GL_TEXTURE_2D);
  glBindTexture(GL_TEXTURE_2D, texture_);

  glBegin(GL_QUADS);
    glTexCoord2d(0.0,0.0); glVertex2d(-1.0,-1.0);
    glTexCoord2d(1.0,0.0); glVertex2d(1.0,-1.0);
    glTexCoord2d(1.0,1.0); glVertex2d(1.0,1.0);
    glTexCoord2d(0.0,1.0); glVertex2d(-1.0,1.0);
  glEnd();
  
  glDisable(GL_TEXTURE_2D);

  swap();
}
