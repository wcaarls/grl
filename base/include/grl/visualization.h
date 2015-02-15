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
    void clear();
    void initProjection(double x1, double x2, double y1, double y2);
    void drawLink(double x1, double y1, double x2, double y2);
    void drawMass(double x, double y);
    void drawJoint(double x, double y);
        
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

    virtual void swap() = 0;
    virtual void clear() = 0;
    virtual void initProjection(double x1, double x2, double y1, double y2) = 0;
    virtual void drawLink(double x1, double y1, double x2, double y2) = 0;
    virtual void drawMass(double x, double y) = 0;
    virtual void drawJoint(double x, double y) = 0;
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
  Visualizer::instance()->swap();
}

inline void Visualization::clear()
{
  Visualizer::instance()->clear();
}

inline void Visualization::initProjection(double x1, double x2, double y1, double y2)
{
  Visualizer::instance()->initProjection(x1, x2, y1, y2);
}

inline void Visualization::drawLink(double x1, double y1, double x2, double y2)
{
  Visualizer::instance()->drawLink(x1, y1, x2, y2);
}

inline void Visualization::drawMass(double x, double y)
{
  Visualizer::instance()->drawMass(x, y);
}

inline void Visualization::drawJoint(double x, double y)
{
  Visualizer::instance()->drawJoint(x, y);
}

} /* namespace grl */

#endif /* GRL_VISUALIZATION_H_ */
