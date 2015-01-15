#ifndef GRL_PENDULUM_H_
#define GRL_PENDULUM_H_

#include <grl/environment.h>

namespace grl
{

class PendulumDynamics : public Dynamics
{
  public:
    TYPEINFO("dynamics/pendulum")

  public:
    double J_, m_, g_, l_, b_, K_, R_;
  
  public:
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Dynamics
    virtual PendulumDynamics *clone() const;
    virtual void eom(const Vector &state, const Vector &action, Vector *xd) const;
};

class PendulumSwingupTask : public Task
{
  public:
    TYPEINFO("task/pendulum/swingup")
  
  public:
    double T_;
  
  public:
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Task
    virtual PendulumSwingupTask *clone() const;
    virtual void start(Vector *state) const;
    virtual void observe(const Vector &state, Vector *obs, int *terminal) const;
    virtual bool evaluate(const Vector &state, const Vector &action, const Vector &next, double *reward) const;
};

}

#endif /* GRL_PENDULUM_H_ */
