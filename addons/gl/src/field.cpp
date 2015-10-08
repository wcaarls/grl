/** \file field.cpp
 * \brief Field visualization source file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-02-14
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

#include <unistd.h>
#include <GL/gl.h>
#include <GL/glu.h>

#include <grl/visualizations/field.h>

#include <libics.h>

#define EPS 0.001

using namespace grl;

void FieldVisualization::request(ConfigurationRequest *config)
{
  config->push_back(CRP("field_dims", "Dimensions to visualize", dims_, CRP::Online));
  config->push_back(CRP("input_min", "Lower input dimension limit", state_min_, CRP::System));
  config->push_back(CRP("input_max", "Upper input dimension limit", state_max_, CRP::System));
  config->push_back(CRP("points", "Number of points to evaluate", points_));
  config->push_back(CRP("savepoints", "Number of points to evaluate when saving to file ('s')", savepoints_));
  
  std::vector<std::string> options;
  options.push_back("mean");
  options.push_back("min");
  options.push_back("max");
  config->push_back(CRP("projection", "Method of projecting values onto 2d space", projection_str_, CRP::Online, options));
}

void FieldVisualization::configure(Configuration &config)
{
  if (!Visualizer::instance())
    throw Exception("visualization/field requires a configured visualizer to run");

  projection_str_ = config["projection"].str();
  if (projection_str_ == "mean")     projection_ = vpMean;
  else if (projection_str_ == "min") projection_ = vpMin;
  else if (projection_str_ == "max") projection_ = vpMax;
  else throw bad_param("visualization/field:projection");
  
  state_min_ = config["input_min"];
  state_max_ = config["input_max"];
  state_dims_ = state_min_.size();
  points_ = config["points"];
  savepoints_ = pow((int)pow(config["savepoints"], 1./state_dims_), state_dims_);

  dims_ = config["field_dims"];
  if (dims_.size() != 2)
    throw bad_param("visualization/field:field_dims");
  
  // Divide points among dimensions
  dimpoints_ = 4*ceil(pow(points_, 1./state_dims_)/4);
  points_ = pow(dimpoints_, state_dims_);
  texpoints_ = dimpoints_*dimpoints_;
  
  // Allocate texture
  data_ = (unsigned char*) malloc(texpoints_*3*sizeof(unsigned char));
  
  DEBUG("Calculating " << dimpoints_ << "x" << dimpoints_ << " (" << texpoints_ << ") texture from " << points_ << " points with spacing " << (state_max_-state_min_)/(dimpoints_-1));}

void FieldVisualization::reconfigure(const Configuration &config)
{
  config.get("projection", projection_str_);
  if (projection_str_ == "mean")     projection_ = vpMean;
  else if (projection_str_ == "min") projection_ = vpMin;
  else if (projection_str_ == "max") projection_ = vpMax;
  else throw bad_param("visualization/field:projection");
  
  config.get("field_dims", dims_);
  if (dims_.size() != 2)
    throw bad_param("visualization/field:field_dims");
}

void FieldVisualization::reshape(int width, int height)
{
  initProjection(-1, 1, -1, 1);
}

void FieldVisualization::key(unsigned char k, int x, int y)
{
  if (k == 's')
    save("grl.ics");
}

void FieldVisualization::save(const std::string &file) const
{
  size_t dimpoints = pow(savepoints_, 1./state_dims_);
  float *field = new float[savepoints_];
  
  const Vector delta = (state_max_-state_min_)/(dimpoints-1);
  
  // Gather data
  Vector ss = state_min_;
  for (int ii=0; ii < savepoints_; ++ii)
  {
    field[ii] = value(ss);
            
    for (int dd=0; dd < state_dims_; ++dd)
    {
      ss[dd] = ss[dd] + delta[dd];
      if (ss[dd] > (state_max_[dd]+EPS))
        ss[dd] = state_min_[dd];
      else
        break;
    }
  }
  
  ICS* ip;
  size_t dims[state_dims_];
  for (int ii=0; ii < state_dims_; ++ii)
    dims[ii] = dimpoints;

  // Write value function
  if (IcsOpen(&ip, file.c_str(), "w2") != IcsErr_Ok)
    throw Exception("Couldn't open ICS file for writing");

  IcsSetLayout(ip, Ics_real32, state_dims_, dims);
  for (int ii=0; ii < state_dims_; ++ii)
  {
    char buf[256]; sprintf(buf, "x%d", ii+1);
    IcsSetOrder(ip, ii, buf, buf);
    IcsSetPosition (ip, ii, state_min_[ii], delta[ii], "relative");
  }
  IcsSetData (ip, (void*)field, savepoints_ * sizeof(float));

  if (IcsClose(ip) != IcsErr_Ok)
    throw Exception("Error closing value ICS file");

  delete[] field;
}

void FieldVisualization::run()
{
  float *field = new float[texpoints_];
  const Vector delta = (state_max_-state_min_)/(dimpoints_-1);
  
  while (ok())
  {
    // Create point iteration order lookup table  
    std::vector<int> dim_order;
    for (int ii=0; ii < state_dims_; ++ii)
      if (ii != dims_[0] && ii != dims_[1])
        dim_order.push_back(ii);
    dim_order.push_back(dims_[0]);
    dim_order.push_back(dims_[1]);

    // Gather data
    Vector ss = state_min_;
    float value_max=-std::numeric_limits<float>::infinity(),
          value_min= std::numeric_limits<float>::infinity();
    
    for (int ii=0; ii < texpoints_; ++ii)
    {
      float v;
    
      switch (projection_)
      {
        case vpMean:
        default:
          v = 0;
          break;
        case vpMin:
          v = std::numeric_limits<float>::infinity();
          break;
        case vpMax:
          v = -std::numeric_limits<float>::infinity();
          break;
      }
    
      for (int jj=0; jj < points_/texpoints_; ++jj)
      {
        switch (projection_)
        {
          case vpMean:
            v += value(ss);
            break;
          case vpMin:
            v = fmin(v, value(ss));
            break;
          case vpMax:
            v = fmax(v, value(ss));
            break;
        }
        
        for (int dd=0; dd < state_dims_; ++dd)
        {
          int oo = dim_order[dd];
      
          ss[oo] = ss[oo] + delta[oo];
          if (ss[oo] > (state_max_[oo]+EPS))
            ss[oo] = state_min_[oo];
          else
            break;
        }
      }

      if (projection_ == vpMean)
        v /= points_/texpoints_;

      field[ii] = v;
      value_max = fmax(v, value_max);
      value_min = fmin(v, value_min);
    }

    DEBUG("Range " << value_min_ << " - " << value_max_);

    float value_range = value_max-value_min;
    
    // Create texture
    for (int ii=0; ii < texpoints_; ++ii)
    {
      double v = (field[ii] - value_min)/value_range;
      double v2 = 4*v;
      
      // Jet colormap
      data_[ii*3+0] = fmax(fmin(255*fmin(v2 - 1.5, -v2 + 4.5), 255), 0);
      data_[ii*3+1] = fmax(fmin(255*fmin(v2 - 0.5, -v2 + 3.5), 255), 0);
      data_[ii*3+2] = fmax(fmin(255*fmin(v2 + 0.5, -v2 + 2.5), 255), 0);
    }
    
    value_min_ = value_min;
    value_max_ = value_max;

    // Redisplay  
    updated_ = true;
    
    // Wait a bit
    usleep(10000);
  }

  delete[] field;
}

void FieldVisualization::idle()
{
  if (updated_)
    refresh();
}

void FieldVisualization::draw()
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
/*  
  char buf[255];
  sprintf(buf, "%8.2f - %8.2f", value_min_, value_max_);
  
  glColor3f(1.0, 1.0, 1.0);
  glRasterPos2f(-1.0, -1.0);
  glutBitmapString(GLUT_BITMAP_9_BY_15, (unsigned char*)buf);
  
  glBegin(GL_LINES);
  glVertex2d(state_[0]-0.05, state_[1]);
  glVertex2d(state_[0]+0.05, state_[1]);
  glVertex2d(state_[0], state_[1]-0.05);
  glVertex2d(state_[0], state_[1]+0.05);
  glEnd();
*/ 
  swap();
}
