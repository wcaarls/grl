/** \file value_function.h
 * \brief Value function visualization header file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-01-22
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

#ifndef GRL_VALUE_FUNCTION_VISUALIZATION_H_
#define GRL_VALUE_FUNCTION_VISUALIZATION_H_

#include <string.h>
#include <pthread.h>

#include <grl/projector.h>
#include <grl/representation.h>
#include <grl/policies/q.h>
#include <grl/visualization.h>

namespace grl
{

/// Value function visualization.
class ValueFunctionVisualization : public Visualization
{
  public:
    TYPEINFO("visualization/value_function")

  protected:
    Projector *projector_;
    Representation *representation_;
    QPolicy *policy_;
  
    int state_dims_;
    Vector state_min_, state_max_, dims_;
    int points_, dimpoints_, texpoints_;
    unsigned int texture_;
    unsigned char *data_;
    Vector dim_order_;
    double value_min_, value_max_;
    bool updated_;
    Vector state_;
  
  public:
    ValueFunctionVisualization() : state_dims_(0), points_(1048576), dimpoints_(0), texpoints_(0), texture_(0), data_(NULL), value_min_(0), value_max_(0), updated_(true)
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

#endif /* GRL_VALUE_FUNCTION_VISUALIZATION_H_ */
