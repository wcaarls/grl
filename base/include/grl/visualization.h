/** \file visualization.h
 * \brief Generic visualization and visualizer definitions.
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

#ifndef GRL_VISUALIZATION_H_
#define GRL_VISUALIZATION_H_

#include <string.h>
#include <pthread.h>

#include <grl/configurable.h>

namespace grl
{

/// Visualization (basically a GL window).
class Visualization : public Configurable
{
  private:
    int id_;

  public:
    virtual ~Visualization();

    void id(int _id) { id_ = _id; }
    int  id() { return id_; }  
    
    void create(const char *name);
    void destroy();
    void refresh();
    void swap();
    
    virtual void draw() { }
    virtual void idle() { }
    virtual void reshape(int width, int height) { }
    virtual void visible(int vis) { }
    virtual void close() { }
};

/// Visualizer (basically a GL window manager).
class Visualizer : public Configurable
{
  protected:
    static Visualizer* instance_;

  public:
    virtual ~Visualizer() { }
    
    // Get singleton instance
    static Visualizer* instance() { return instance_; }
    
    virtual void createWindow(Visualization *window, const char *name) = 0;
    virtual void destroyWindow(Visualization *window, bool glutDestroy=true) = 0;
    virtual void refreshWindow(Visualization *window) = 0;
    virtual void swapWindow(Visualization *window) = 0;
};

inline Visualization::~Visualization()
{
  destroy();
}

inline void Visualization::create(const char *name)
{
  Visualizer::instance()->createWindow(this, (char*)name);
}

inline void Visualization::destroy()
{
  Visualizer::instance()->destroyWindow(this);
}

inline void Visualization::refresh()
{
  Visualizer::instance()->refreshWindow(this);
}

inline void Visualization::swap()
{
  Visualizer::instance()->swapWindow(this);
}

} /* namespace grl */

#endif /* GRL_VISUALIZATION_H_ */
