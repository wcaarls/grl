/** \file slice.cpp
 * \brief Slice function visualization source file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2016-12-02
 *
 * \copyright \verbatim
 * Copyright (c) 2016, Wouter Caarls
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

#include <grl/visualizations/slice.h>

#include <libics.h>

#define EPS 0.001

using namespace grl;

REGISTER_CONFIGURABLE(SliceVisualization) 

void SliceVisualization::request(ConfigurationRequest *config)
{
  config->push_back(CRP("field_dims", "Dimensions to visualize", dims_, CRP::Online));
  config->push_back(CRP("input_min", "Lower input dimension limit", state_min_, CRP::System));
  config->push_back(CRP("input_max", "Upper input dimension limit", state_max_, CRP::System));
  config->push_back(CRP("operating_point", "Fixed values for non-visualized dimensions", operating_point_, CRP::Online));
  config->push_back(CRP("output_dim", "Output dimension to visualize", (int)dim_, CRP::Online, 0));
  config->push_back(CRP("points", "Number of points to evaluate", points_));
  config->push_back(CRP("savepoints", "Number of points to save when pressing 'i'", savepoints_));
  config->push_back(CRP("delay", "Delay between updates (s)", delay_, CRP::Online, 0., 3600.));
  config->push_back(CRP("state", "signal/vector", "Optional current state to overlay", state_, true));
  config->push_back(CRP("action", "signal/vector", "Optional current action to overlay", action_, true));
  config->push_back(CRP("mapping", "mapping", "Mapping to visualize", mapping_));
}

void SliceVisualization::configure(Configuration &config)
{
  if (!Visualizer::instance())
  {
    WARNING("visualization/slice requires a configured visualizer to run");
    return;
  }

  dims_ = config["field_dims"].v();
  if (dims_.size() != 2)
    throw bad_param("visualization/slice:field_dims");

  state_min_ = config["input_min"].v();
  state_max_ = config["input_max"].v();
  state_dims_ = state_min_.size();
  
  operating_point_ = config["operating_point"].v();
  if (!operating_point_.size())
    operating_point_ = (state_min_+state_max_)/2;
  if (operating_point_.size() != state_dims_)
    throw bad_param("visualization/slice:operating_point");
  
  dim_ = config["output_dim"];
  points_ = pow((int)sqrt((double)config["points"]), 2);
  savepoints_ = pow((int)pow((double)config["savepoints"], 1./state_dims_), state_dims_);
  delay_ = config["delay"];
  state_ = (VectorSignal*)config["state"].ptr();
  action_ = (VectorSignal*)config["action"].ptr();
  mapping_ = (Mapping*)config["mapping"].ptr();
  
  // Allocate texture
  data_ = (unsigned char*) malloc(points_*3*sizeof(unsigned char));

  // Create window  
  create(path().c_str());
  
  // Let's get this show on the road
  start();
}

void SliceVisualization::reconfigure(const Configuration &config)
{
  config.get("field_dims", dims_);
  if (dims_.size() != 2)
    throw bad_param("visualization/slice:field_dims");

  config.get("operating_point", operating_point_);
  if (operating_point_.size() != state_dims_)
    throw bad_param("visualization/slice:operating_point");

  config.get("output_dim", dim_);
  config.get("delay", delay_);
}

void SliceVisualization::reshape(int width, int height)
{
  initProjection(0, 1, 0, 1);
}

void SliceVisualization::key(unsigned char k, int x, int y)
{
  if (k == 'i')
  {
    save("slice.ics");
    return;
  }

  switch (k)
  {
    case 'w':
      dims_[1] = ((int)dims_[1]+1)%state_dims_;
      break;
    case 'a':
      dims_[0] = ((int)dims_[0]-1+state_dims_)%state_dims_;
      break;
    case 's':
      dims_[1] = ((int)dims_[1]-1+state_dims_)%state_dims_;
      break;
    case 'd':
      dims_[0] = ((int)dims_[0]+1)%state_dims_;
      break;
  }
  
  if (k >= '0' && k <= '9')
    dim_ = k-'0';
  
  INFO("Now visualizing " << dims_ << " -> " << dim_);
}

void SliceVisualization::click(int button, int state, int x, int y)
{
  if (state != 0)
    return;

  double ox, oy, oz;
  double model[16], proj[16];
  int view[4];

  glGetDoublev(GL_MODELVIEW_MATRIX, model);
  glGetDoublev(GL_PROJECTION_MATRIX, proj);
  glGetIntegerv(GL_VIEWPORT, view);
  gluUnProject(x, y, 0, model, proj, view, &ox, &oy, &oz);

  const Vector range = (state_max_-state_min_);
  size_t dimx = dims_[0], dimy = dims_[1];
  
  double xx = state_min_[dimx] + ox*range[dimx], yy = state_max_[dimy] - oy*range[dimy];
  
  Vector op = operating_point_, r;
  op[dimx] = xx;
  op[dimy] = yy;
  
  switch (button)
  {
    case 0:
      mapping_->read(op, &r);
      INFO("Value at " << op << ": " << r[dim_]);
      break;
    case 2:
      operating_point_ = op;
      INFO("Operating point now " << operating_point_);
      break;
  }
}

void SliceVisualization::save(const std::string &file) const
{
  size_t dimpoints = pow(savepoints_, 1./state_dims_);
  const Vector delta = (state_max_-state_min_)/(dimpoints-1);
  
  // Construct query matrix
  Matrix query(savepoints_, operating_point_.size());
  Vector ss = state_min_;
  for (int ii=0; ii < savepoints_; ++ii)
  {
    query.row(ii) = ss;
            
    for (int dd=0; dd < state_dims_; ++dd)
    {
      ss[dd] = ss[dd] + delta[dd];
      if (ss[dd] > (state_max_[dd]+EPS))
        ss[dd] = state_min_[dd];
      else
        break;
    }
  }

  // Don't do too many at once
  const size_t block_size = 262144;
  float *field = NULL;
  size_t outputs=0;
  
  for (size_t nn=0; nn < savepoints_; nn += block_size)
  {
    // Gather data
    Matrix result;
    mapping_->read(query.block(nn, 0, std::min(block_size, savepoints_-nn), query.cols()), &result);
    
    if (!field)
    {
      outputs = result.cols();
      field = new float[savepoints_*outputs];
    }
  
    // Convert to float
    for (size_t ii=0; ii < result.rows(); ++ii)
      for (size_t jj=0; jj < result.cols(); ++jj)
        field[(nn+ii)*outputs+jj] = result(ii, jj);
  }

  ICS* ip;
  size_t dims[state_dims_+1];
  dims[0] = outputs;
  for (int ii=0; ii < state_dims_; ++ii)
    dims[ii+1] = dimpoints;

  // Write value function
  if (IcsOpen(&ip, file.c_str(), "w2") != IcsErr_Ok)
    throw Exception("Couldn't open ICS file for writing");

  IcsSetLayout(ip, Ics_real32, state_dims_+1, dims);
  IcsSetOrder(ip, 0, "tensor", "tensor");
  for (int ii=0; ii < state_dims_; ++ii)
  {
    char buf[256]; sprintf(buf, "x%d", ii);
    IcsSetOrder(ip, ii+1, buf, buf);
    IcsSetPosition(ip, ii+1, state_min_[ii], delta[ii], "relative");
  }
  
  IcsSetData (ip, (void*)field, savepoints_ * outputs * sizeof(float));

  if (IcsClose(ip) != IcsErr_Ok)
    throw Exception("Error closing value ICS file");

  delete[] field;
}

void SliceVisualization::run()
{
  int dimpoints = sqrt(points_);
  const Vector delta = (state_max_-state_min_)/(dimpoints-1);
  size_t dimx = dims_[0], dimy = dims_[1];
  
  // Construct query matrix
  Matrix query(dimpoints*dimpoints, operating_point_.size());
  Vector ss = operating_point_;
  
  for (int yy=0, ii=0; yy < dimpoints; ++yy)
  {
    ss[dimy] = state_min_[dimy]+yy*delta[dimy];
    
    for (int xx=0; xx < dimpoints; ++xx, ++ii)
    {
      ss[dimx] = state_min_[dimx]+xx*delta[dimx];
      query.row(ii) = ss;
    }
  }
  
  while (ok())
  {
    float value_max=-std::numeric_limits<float>::infinity(),
          value_min= std::numeric_limits<float>::infinity();
    
    // Gather data
    Matrix result;
    mapping_->read(query, &result);
    value_min = result.col(dim_).minCoeff();
    value_max = result.col(dim_).maxCoeff();

    TRACE("Range " << value_min_ << " - " << value_max_);

    float value_range = value_max-value_min;
    
    // Create texture
    for (int ii=0; ii < points_; ++ii)
    {
      double v = (result(ii, dim_) - value_min)/value_range;
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
    usleep(delay_*1000000);
  }
}

void SliceVisualization::idle()
{
  if (updated_ || (state_ && state_->test()))
    refresh();
}

void SliceVisualization::draw()
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
    
    gluBuild2DMipmaps( GL_TEXTURE_2D, 3, sqrt(points_), sqrt(points_), GL_RGB, GL_UNSIGNED_BYTE, data_ );
    
    updated_ = false;
  }

  clear();
  
  if (texture_)
  {
    // Draw texture
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, texture_);

    glBegin(GL_QUADS);
      glTexCoord2d(0.0,0.0); glVertex2d(0.0,0.0);
      glTexCoord2d(1.0,0.0); glVertex2d(1.0,0.0);
      glTexCoord2d(1.0,1.0); glVertex2d(1.0,1.0);
      glTexCoord2d(0.0,1.0); glVertex2d(0.0,1.0);
    glEnd();
  
    glDisable(GL_TEXTURE_2D);
  }
  
  if (state_)
  {
    Vector state = state_->get();
    if (action_)
      state = extend(state, action_->get());
    if (state.size() < state_dims_)
      state = extend(state, ConstantVector(state_dims_-state.size(), 0.));
    
    if (state.size())
    {
      double x = (state[dims_[0]] - state_min_[dims_[0]]) / (state_max_[dims_[0]] - state_min_[dims_[0]]);
      double y = (state[dims_[1]] - state_min_[dims_[1]]) / (state_max_[dims_[1]] - state_min_[dims_[1]]);

      glBegin(GL_LINES);
      glVertex2d(x-0.05, y);
      glVertex2d(x+0.05, y);
      glVertex2d(x, y-0.05);
      glVertex2d(x, y+0.05);
      glEnd();
    }
  }

  // Ugly hack to avoid swapping in a derived class
  if (d_type() == s_type())
    swap();
}
