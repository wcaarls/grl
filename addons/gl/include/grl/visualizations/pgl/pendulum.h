/** \file pendulum.h
 * \brief PGL-based pendulum visualization header file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2020-06-23
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

#ifndef GRL_PGL_PENDULUM_VISUALIZATION_H_
#define GRL_PGL_PENDULUM_VISUALIZATION_H_

#include <grl/visualizations/pgl.h>

namespace grl
{

/// Pendulum visualization.
class PGLPendulumVisualization : public PGLVisualization
{
  public:
    TYPEINFO("visualization/pgl/pendulum", "PGL-based pendulum visualization")

  protected:
    pgl::Object *pendulum_;
  
  public:
    PGLPendulumVisualization() : pendulum_(NULL) { }
    
  protected:
    // From PGLVisualization
    virtual void createScene();
    virtual void updateScene(const Vector &state);
};

}

#endif /* GRL_PGL_PENDULUM_VISUALIZATION_H_ */
