/** \file quadcopter.h
 * \brief Quadcopter visualization header file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2020-04-24
 *
 * \copyright \verbatim
 * Copyright (c) 2020, Wouter Caarls
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

#ifndef GRL_QUADCOPTER_VISUALIZATION_H_
#define GRL_QUADCOPTER_VISUALIZATION_H_

#include <grl/visualizations/pgl.h>

namespace grl
{

/// Quadcopter visualization.
class QuadcopterVisualization : public PGLVisualization
{
  public:
    TYPEINFO("visualization/quadcopter", "Quadcopter visualization")

  protected:
    Vector limits_;
    
    pgl::Object *quadcopter_;
  
  public:
    QuadcopterVisualization() : quadcopter_(NULL)
    {
      limits_ = VectorConstructor(1, 0);
    }
    
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
  
  protected:
    // From PGLVisualization
    virtual void createScene();
    virtual void updateScene(const Vector &state);
};

}

#endif /* GRL_QUADCOPTER_VISUALIZATION_H_ */
