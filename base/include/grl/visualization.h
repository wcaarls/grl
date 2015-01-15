#ifndef GRL_VISUALIZATION_H_
#define GRL_VISUALIZATION_H_

#include <string.h>
#include <pthread.h>

#include <grl/configurable.h>

namespace grl
{

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
