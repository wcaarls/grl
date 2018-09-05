/** \file slice.h
 * \brief Slice visualization header file.
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

#ifndef GRL_SLICE_VISUALIZATION_H_
#define GRL_SLICE_VISUALIZATION_H_

#include <string.h>

#include <itc/itc.h>
#include <grl/visualization.h>
#include <grl/signal.h>
#include <grl/mapping.h>

namespace grl
{

/// Slice visualization base class.
class SliceVisualization : public Visualization, public itc::Thread
{
  public:
    TYPEINFO("visualization/slice", "Visualizes a slice from a mapping");

  protected:
    Vector dims_;
    Vector state_min_, state_max_;
    int state_dims_;
    Vector operating_point_;
    int dim_, points_;
    double delay_;
    VectorSignal *state_, *action_;
    Mapping *mapping_;
    
    unsigned int texture_;
    unsigned char *data_;
    double value_min_, value_max_;
    bool updated_;
  
  public:
    SliceVisualization() : state_dims_(0), dim_(0), points_(65536), delay_(0.1), state_(NULL), action_(NULL), mapping_(NULL), texture_(0), data_(NULL), value_min_(0), value_max_(0), updated_(true)
    {
      dims_ = VectorConstructor(0., 1.);
    }
    
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);
  
    // From Visualization
    virtual void draw(); 
    virtual void idle();
    virtual void reshape(int width, int height);
    virtual void key(unsigned char k, int x, int y);
    virtual void click(int button, int state, int x, int y);
    
    // From itc::Thread
    virtual void run();
};

}

#endif /* GRL_SLICE_VISUALIZATION_H_ */
