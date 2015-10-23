/** \file field.h
 * \brief Field visualization header file.
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

#ifndef GRL_FIELD_VISUALIZATION_H_
#define GRL_FIELD_VISUALIZATION_H_

#include <string.h>

#include <itc/itc.h>
#include <grl/visualization.h>
#include <grl/state.h>

namespace grl
{

/// Field visualization base class.
class FieldVisualization : public Visualization, public itc::Thread
{
  protected:
    enum ValueProjection { vpMean, vpMin, vpMax };
  
    State *state_;
    int state_dims_;
    Vector state_min_, state_max_, dims_;
    int points_, savepoints_, dimpoints_, texpoints_;
    unsigned int texture_;
    unsigned char *data_;
    double value_min_, value_max_;
    bool updated_;
    std::string projection_str_;
    ValueProjection projection_;
  
  public:
    FieldVisualization() : state_(NULL), state_dims_(0), points_(65536), savepoints_(1048576), dimpoints_(0), texpoints_(0), texture_(0), data_(NULL), value_min_(0), value_max_(0), updated_(true), projection_str_("mean"), projection_(vpMean)
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
    
    // From itc::Thread
    virtual void run();
    
  protected:
    virtual double value(const Vector &in) const = 0;
    virtual void save(const std::string &file) const;
};

}

#endif /* GRL_FIELD_VISUALIZATION_H_ */
