#ifndef ODESIM_POLICY_PLAYER_H_
#define ODESIM_POLICY_PLAYER_H_

#include <QtGui>

#include <GenericState.h>
#include <STGListener.h>

#include <itc/itc.h>
#include <grl/environment.h>

#include "simulator.h"

namespace grl
{

class ODESTGEnvironment: public QObject
{
  Q_OBJECT

  protected:
    ODESimulator                   simulator_;
    CSTGListener<GenericState>     listener_;
    
    std::vector<CGenericStateVar>  sensors_;
    std::vector<CGenericActionVar> actuators_;
    CGenericStateVar               termination_, reward_;
    
    uint64_t                       start_time_, timeout_;
    
  public:
    ODESTGEnvironment() : listener_(&simulator_), timeout_(0) { }
    ~ODESTGEnvironment();
  
    bool configure(Configuration &config);
    void start(int test, Vector *obs);
    double step(const Vector &action, Vector *obs, double *reward, int *terminal);
    
    ODESimulator *getSim() { return &simulator_; }

  signals:
    void drawFrame();  // signal to emit to the GUI to initiate drawing of the screen
};

class ODEEnvironment: public grl::Environment, public itc::Thread
{
  public:
    TYPEINFO("environment/ode", "Open Dynamics Engine simulation environment")

  protected:
    QApplication *app_;
    ODESTGEnvironment *env_;
    Configuration *config_;
    std::string xml_;
    int visualize_;
    bool initialized_;

  public:
    ODEEnvironment() : app_(NULL), env_(NULL), config_(NULL), xml_("../addons/odesim/cfg/robot.xml"), visualize_(1), initialized_(false) { }
    ~ODEEnvironment();
  
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);
    
    // From Environment
    virtual ODEEnvironment *clone() const
    {
      return NULL;
    }

    virtual void start(int test, Vector *obs)
    {
      env_->start(test, obs);
    }
    
    virtual double step(const Vector &action, Vector *obs, double *reward, int *terminal)
    {
      return env_->step(action, obs, reward, terminal);
    }
    
    // From Thread
    virtual void run();
};

}

#endif /* ODESIM_POLICY_PLAYER_H_ */
