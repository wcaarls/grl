#ifndef GRL_LEOSIM_ENVIRONMENT_H_
#define GRL_LEOSIM_ENVIRONMENT_H_

#include <grl/environments/odesim/environment.h>

namespace grl
{

/// Simulation of original Leo robot by Erik Schuitema.
class LeoSimEnvironment : public ODEEnvironment
{
  public:
    TYPEINFO("environment/leosim", "Leo simulation environment")

  public:
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);
    
    // From ODEEnvironment
    virtual LeoSimEnvironment *clone();
    
    // From Environment
    virtual void start(int test, Vector *obs);
    virtual double step(const Vector &action, Vector *obs, double *reward, int *terminal);
};

}

#endif /* GRL_LEOSIM_ENVIRONMENT_H_ */
