/** \file glut.h
 * \brief GLUT visualizer header file.
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

#ifndef GRL_GLUT_H_
#define GRL_GLUT_H_

#include <string.h>
#include <pthread.h>

#include <grl/visualization.h>

namespace grl
{

/// Visualizer based on the GLUT library.
class GLUTVisualizer : public Visualizer
{
  public:
    TYPEINFO("visualizer/glut", "Visualizer based on the GLUT library")

  protected:
    pthread_t thread_;
    pthread_mutex_t mutex_;
    bool continue_;
    int window_;
  
    typedef std::map<int, Visualization*> WindowMap;
    WindowMap windows_;
    
    const char *new_window_name_;
    Visualization *new_window_ptr_;

  public:
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);
    
    // From Visualizer
    virtual void createWindow(Visualization *window, const char *name);
    virtual void destroyWindow(Visualization *window);
    virtual void refreshWindow(Visualization *window);
    
    virtual void setTitle(const char *name);
    virtual void swap();
    virtual void clear();
    virtual void initProjection(double x1, double x2, double y1, double y2);
    virtual void drawLink(double x1, double y1, double x2, double y2);
    virtual void drawMass(double x, double y);
    virtual void drawJoint(double x, double y);
    virtual void drawSurface(double x1, double y1, double x2, double y2, double r=1., double g=1., double b=1.);
    virtual void drawTexture(double x1, double y1, double x2, double y2, unsigned char *data, int width, int height, bool colored=false);
    
  protected:    
    ~GLUTVisualizer()
    {
      if (continue_)
      {
        continue_ = false;
        pthread_join(thread_, NULL);
      }
    }
    
    void run();
    static GLUTVisualizer *glutInstance() { return dynamic_cast<GLUTVisualizer*>(instance()); }
    Visualization *getCurrentWindow();
    void destroyWindow(Visualization *window, bool glutDestroy);
    
    // Delegates

    static void *run(void *)
    {
      glutInstance()->run();
      return NULL;
    }
    
    static void idle()
    {
      GLUTVisualizer *driver = glutInstance();
      
      for (WindowMap::iterator it=driver->windows_.begin(); it != driver->windows_.end(); ++it)
        it->second->idle();
    }
    
    static void draw()
    {
      Visualization* window = glutInstance()->getCurrentWindow();
      if (window)
        window->draw();
    }
    
    static void reshape(int width, int height) 
    {
      glViewport(0, 0, width, height);
    
      Visualization* window = glutInstance()->getCurrentWindow();
      if (window)
        window->reshape(width, height);
    }
    
    static void visible(int vis)
    {
      Visualization* window = glutInstance()->getCurrentWindow();
      if (window)
        window->visible(vis);
    }
    
    static void close()
    {
      Visualization* window = glutInstance()->getCurrentWindow();
      if (window)
      {
        glutInstance()->destroyWindow(window, false);
        window->close();
      }
    }

    static void key(unsigned char k, int x, int y)
    {
      Visualization* window = glutInstance()->getCurrentWindow();
      if (window)
        window->key(k, x, y);
    }
    
    static void click(int button, int state, int x, int y)
    {
      Visualization* window = glutInstance()->getCurrentWindow();
      if (window)
        window->click(button, state, x, y);
    }
    
    static void motion(int x, int y)
    {
      Visualization* window = glutInstance()->getCurrentWindow();
      if (window)
        window->motion(x, y);
    }
    
    static void error(const char *fmt, va_list ap)
    {
      char errmsg[PATH_MAX];
      std::ostringstream oss;
      
      vsnprintf(errmsg, PATH_MAX, fmt, ap);
      oss << errmsg;
      grl::log(0, oss);
      pthread_mutex_unlock(&glutInstance()->mutex_);
      glutInstance()->instance_ = NULL;
      pthread_exit(NULL);
    }
};

}

#endif /* GRL_GLUT_H_ */
