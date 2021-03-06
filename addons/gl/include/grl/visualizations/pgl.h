/** \file pgl.h
 * \brief PGL visualization header file.
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

#ifndef GRL_PGL_VISUALIZATION_H_
#define GRL_PGL_VISUALIZATION_H_

#include <grl/signal.h>
#include <grl/visualization.h>

#include <pgl/pgl.h>

namespace grl
{

/// PGL visualization.
class PGLVisualization : public Visualization
{
  protected:
    VectorSignal *state_;
  
    pgl::Scene *scene_;
    pgl::Camera *camera_;
    pgl::OrbitController *controller_;
  
  public:
    PGLVisualization() : state_(NULL), scene_(NULL), camera_(NULL), controller_(NULL) { }
    
    ~PGLVisualization()
    {
      if (controller_)
        delete controller_;
      if (camera_)
        delete camera_;
      if (scene_)
        delete scene_;
    }
    
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);
  
    // From Visualization
    virtual void draw();
    virtual void idle();
    virtual void reshape(int width, int height);
    virtual void click(int button, int state, int x, int y);
    virtual void motion(int x, int y);
    
  protected:
    virtual void createScene() = 0;
    virtual void updateScene(const Vector &state) = 0;
};

}

#endif /* GRL_PGL_VISUALIZATION_H_ */
