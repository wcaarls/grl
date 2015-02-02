/** \file glut.cpp
 * \brief GLUT visualizer source file.
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

#include <GL/freeglut.h>

#include <grl/visualizers/glut.h>

#define EPS 0.001

using namespace grl;

REGISTER_CONFIGURABLE(GLUTVisualizer)

void GLUTVisualizer::request(ConfigurationRequest *config)
{
}

void GLUTVisualizer::configure(Configuration &config)
{
  NOTICE("Registering GLUT visualization engine");
  instance_ = this;

  continue_ = true;
  pthread_mutex_init(&mutex_, NULL);
  pthread_mutex_lock(&mutex_);
  pthread_create(&thread_, NULL, run, NULL);
}

void GLUTVisualizer::reconfigure(const Configuration &config)
{
}

void GLUTVisualizer::createWindow(Visualization *window, const char *name)
{
  pthread_mutex_lock(&mutex_);
  
  new_window_name_ = name;
  new_window_ptr_  = window;
  
  while (new_window_name_)
  {
    pthread_mutex_unlock(&mutex_);
    usleep(0);
    pthread_mutex_lock(&mutex_);
  }
  
  pthread_mutex_unlock(&mutex_);
}
    
void GLUTVisualizer::destroyWindow(Visualization *window, bool glutDestroy)
{
  pthread_mutex_lock(&mutex_);
      
  WindowMap::iterator it=windows_.begin();
      
  while (it != windows_.end())
  {
    if (it->second == window)
    {
      if (glutDestroy)
        glutDestroyWindow(it->first);
      windows_.erase(it++);
    }
    else
      ++it;
  }
      
  pthread_mutex_unlock(&mutex_);
}

void GLUTVisualizer::refreshWindow(Visualization *window)
{
  glutPostWindowRedisplay(window->id());
}

void GLUTVisualizer::swapWindow(Visualization *window)
{
  // Window must be current context
  glutSwapBuffers();
}
    
void GLUTVisualizer::run()
{
  int argc = 1;
  char argv1[256], *argv[]={argv1};
      
  strncpy(argv1, "GLUTVisualizer", 256);
  
  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);
  glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_CONTINUE_EXECUTION);
  glutIdleFunc(idle);
  
  pthread_mutex_unlock(&mutex_);
      
  while (continue_)
  {
    pthread_mutex_lock(&mutex_);

    glutMainLoopEvent();
    
    if (new_window_name_)
    {
      glutCreateWindow((char*)new_window_name_);
      glutReshapeWindow(512, 512);
      glutPositionWindow(0, 512);
     
      glutDisplayFunc(draw);
      glutReshapeFunc(reshape);
      glutVisibilityFunc(visible);  
      glutCloseFunc(close);
    
      new_window_ptr_->id(glutGetWindow());
      windows_[new_window_ptr_->id()] = new_window_ptr_;

      new_window_name_ = NULL;
      new_window_ptr_ = NULL;
      
      pthread_mutex_unlock(&mutex_);
    }
    
    pthread_mutex_unlock(&mutex_);
    
    idle();
    usleep(1000);
  }
}
    
Visualization *GLUTVisualizer::getCurrentWindow()
{
  WindowMap::iterator it = windows_.find(glutGetWindow());
  if (it != windows_.end())
    return it->second;
  else
    return NULL;
}
