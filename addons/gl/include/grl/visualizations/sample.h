/** \file sample.h
 * \brief Sample visualization header file.
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

#ifndef GRL_SAMPLE_VISUALIZATION_H_
#define GRL_SAMPLE_VISUALIZATION_H_

#include <string.h>
#include <pthread.h>

#include <grl/projector.h>
#include <grl/visualization.h>

namespace grl
{

/// Sample visualization.
class SampleVisualization : public Visualization
{
  public:
    TYPEINFO("visualization/sample", "Visualizes a sample-based approximation")

  protected:
    SampleProjector *projector_;
  
    int dim_;
    Vector dims_, min_, max_;
    int points_, dimpoints_;
    unsigned int texture_;
    unsigned char *data_;
    double value_min_, value_max_;
    bool updated_;
  
  public:
    SampleVisualization() : projector_(NULL), dim_(0), points_(65536), dimpoints_(0), texture_(0), data_(NULL), value_min_(0), value_max_(0), updated_(true)
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
};

}

#endif /* GRL_SAMPLE_VISUALIZATION_H_ */
