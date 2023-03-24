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
#include <grl/visualization.h>

namespace grl
{

/// Wheeled mobile robot visualization.
class WMRVisualization : public Visualization
{
  public:
    TYPEINFO("visualization/wmr", "Wheeled mobile robot visualization")

  protected:
    VectorSignal *state_;
    double t_, l_, b_;
  
  public:
    WMRVisualization() : state_(NULL) { }
    
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
