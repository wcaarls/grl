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
    Visualization() : id_(-1) { }
    virtual ~Visualization();

    void id(int _id) { id_ = _id; }
    int  id() { return id_; }  
    
    /// Create window.
    void create(const char *name);
    
    /// Destroy window.
    void destroy();
    
    /// Refresh graphics (causes draw() to be called).
    void refresh();
    
    /// Change the window title.
    void setTitle(const char *name);
    
    /// Swap buffers. May only be called from draw() callback.
    void swap();
    
    /// Clear background. May only be called from draw() callback.
    void clear();
    
    /// Set drawing size. May only be called from a callback.
    void initProjection(double x1, double x2, double y1, double y2);
    
    /// Draw a mechanical link. May only be called from draw() callback.
    void drawLink(double x1, double y1, double x2, double y2);
    
    /// Draw a center of mass. May only be called from draw() callback.
    void drawMass(double x, double y);
    
    /// Draw a mechanical joint. May only be called from draw() callback.
    void drawJoint(double x, double y);
        
    /// Draw a surface. May only be called from draw() callback.
    void drawSurface(double x1, double x2, double y1, double y2, double r=1, double g=1, double b=1);

    /// Draw a texture. May only be called from draw() callback.
    void drawTexture(double x1, double y1, double x2, double y2, unsigned char *data, int width, int height, bool colored=false);

    /// Callback that draws the visualization.
    virtual void draw() { }
    
    /// Callback that is called periodically to allow for animation.
    virtual void idle() { }
    
    /// Callback that is called when the window shape is changed.
    virtual void reshape(int width, int height) { }
    
    /// Callback that is called when the window visibility changes.
    virtual void visible(int vis) { }
    
    /// Callback that is called when the window is closed.
    virtual void close() { }

    /// Callback that is called when a key is pressed.
    virtual void key(unsigned char k, int x, int y) { }
    
    /// Callback that is called when a mouse button is clicked.
    virtual void click(int button, int state, int x, int y) { }

    /// Callback that is called when the mouse is moved while holding a button.
    virtual void motion(int x, int y) { }
};

/// Visualizer (basically a GL window manager).
class Visualizer : public Configurable
{
  protected:
    static Visualizer* instance_;

  public:
    virtual ~Visualizer() { }
    
    /// Get singleton instance
    static Visualizer* instance() { return instance_; }
    
    /// Called by a Visualization to create a window for it.
    virtual void createWindow(Visualization *window, const char *name) = 0;
    
    /// Called by a Visualization to destroy its window.
    virtual void destroyWindow(Visualization *window) = 0;
    
    /// Called by a Visualization to refresh its window.
    virtual void refreshWindow(Visualization *window) = 0;
    
    /// Called by a Visualization to change the window title.
    virtual void setTitle(const char *name) = 0;
    
    /// Called by a Visualization to swap buffers.
    virtual void swap() = 0;
    
    /// Called by a Visualization to clear background.
    virtual void clear() = 0;

    /// Called by a Visualization to set drawing size.
    virtual void initProjection(double x1, double x2, double y1, double y2) = 0;

    /// Called by a Visualization to draw a mechanical link.
    virtual void drawLink(double x1, double y1, double x2, double y2) = 0;

    /// Called by a Visualization to draw a center of mass.
    virtual void drawMass(double x, double y) = 0;

    /// Called by a Visualization to draw a mechanical joint.
    virtual void drawJoint(double x, double y) = 0;

    /// Called by a Visualization to draw a surface.
    virtual void drawSurface(double x1, double y1, double x2, double y2, double r=1, double g=1, double b=1) = 0;

    /// Called by a Visualization to draw a texture.
    virtual void drawTexture(double x1, double y1, double x2, double y2, unsigned char *data, int width, int height, bool colored=false) = 0;
};

inline Visualization::~Visualization()
{
  if (id_ != -1)
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

inline void Visualization::setTitle(const char *name)
{
  Visualizer::instance()->setTitle(name);
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

inline void Visualization::drawSurface(double x1, double y1, double x2, double y2, double r, double g, double b)
{
  Visualizer::instance()->drawSurface(x1, y1, x2, y2, r, g, b);
}

inline void Visualization::drawTexture(double x1, double y1, double x2, double y2, unsigned char *data, int width, int height, bool colored)
{
  Visualizer::instance()->drawTexture(x1, y1, x2, y2, data, width, height, colored);
}


} /* namespace grl */

#endif /* GRL_VISUALIZATION_H_ */
