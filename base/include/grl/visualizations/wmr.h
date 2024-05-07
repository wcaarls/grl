/** \file wmr.h
 * \brief Wheeled mobile robot visualization header file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2023-03-13
 *
 * \copyright \verbatim
 * Copyright (c) 2023, Wouter Caarls
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

#ifndef GRL_WMR_VISUALIZATION_H_
#define GRL_WMR_VISUALIZATION_H_

#include <grl/signal.h>
#include <grl/mapping.h>
#include <grl/visualization.h>

namespace grl
{

/// Wheeled mobile robot visualization.
class WMRVisualization : public Visualization
{
  public:
    TYPEINFO("visualization/wmr", "Wheeled mobile robot visualization")

  protected:
    double t_, b_, l_;
    double sensor_pos_, sensor_width_;
    VectorSignal *state_;
    Mapping *trajectory_;
    Vector min_, max_;
    unsigned char *data_;
  
  public:
    WMRVisualization() : t_(1.0), b_(1.0), l_(0.2), sensor_pos_(0.1), sensor_width_(0.1), state_(NULL), trajectory_(NULL), data_(NULL) { }
    
    ~WMRVisualization()
    {
      if (data_)
      {
        delete data_;
        data_ = NULL;
      }
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

#endif /* GRL_WMR_VISUALIZATION_H_ */
