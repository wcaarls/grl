#ifndef GRL_GLUT_H_
#define GRL_GLUT_H_

#include <string.h>
#include <pthread.h>

#include <grl/visualization.h>

namespace grl
{

class GLUTVisualizer : public Visualizer
{
  public:
    TYPEINFO("visualizer/glut")

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
    void createWindow(Visualization *window, const char *name);
    void destroyWindow(Visualization *window, bool glutDestroy=true);
    void refreshWindow(Visualization *window);
    void swapWindow(Visualization *window);
    
  protected:    
    ~GLUTVisualizer()
    {
      continue_ = false;
      pthread_join(thread_, NULL);
    }
    
    void run();
    static GLUTVisualizer *glutInstance() { return dynamic_cast<GLUTVisualizer*>(instance()); }
    Visualization *getCurrentWindow();
    
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
};

} /* namespace grl */

#endif /* GRL_GLUT_H_ */
