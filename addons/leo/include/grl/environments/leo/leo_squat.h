#ifndef GRL_LEO_SQUAT_ENVIRONMENT_H_
#define GRL_LEO_SQUAT_ENVIRONMENT_H_

#include <leo.h>
#include <grl/environments/odesim/environment.h>
#include <LeoBhWalkSym.h>
#include <STGLeo.h>
#include <STGLeoSim.h>
#include <ThirdOrderButterworth.h>

namespace grl
{

class CLeoBhSquat: public CLeoBhBase
{
  public:
    CLeoBhSquat(ISTGActuation *actuationInterface) : CLeoBhBase(actuationInterface) {}
    double calculateReward();
};

/// Squatting Leo robot
class LeoSquatEnvironment: public LeoBaseEnvironment
{
  public:
    TYPEINFO("environment/leo_squat", "Leo squatting environment")

  public:
    LeoSquatEnvironment();
    ~LeoSquatEnvironment() {}

    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);

    // From Environment
    virtual LeoSquatEnvironment *clone() const;
    virtual void start(int test, Vector *obs);
    virtual double step(const Vector &action, Vector *obs, double *reward, int *terminal);

  protected:
    int requested_action_dims_;

  protected:
    void config_parse_actions(Configuration &config);
};

}

#endif /* GRL_LEO_SQUAT_ENVIRONMENT_H_ */
