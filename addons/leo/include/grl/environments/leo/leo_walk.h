#ifndef GRL_LEO_WALK_ENVIRONMENT_H_
#define GRL_LEO_WALK_ENVIRONMENT_H_

#include <leo.h>
#include <grl/environments/odesim/environment.h>
#include <grl/signals/signal_v.h>
#include <LeoBhWalkSym.h>
#include <STGLeo.h>
#include <STGLeoSim.h>
#include <ThirdOrderButterworth.h>

namespace grl
{

class CLeoBhWalk: public CLeoBhBase
{
  public:
    TYPEINFO("behavior/leo_switching_walk", "Leo walking behavior with symmetrical switchers of observations")

    CLeoBhWalk() {}
    virtual double calculateReward();
    virtual void parseLeoState(const CLeoState &leoState, Vector &obs);
    virtual void parseLeoAction(const Vector &action, Vector &target_action);
    virtual std::string getProgressReport(double trialTime);
};
/*
class CLeoBhWalkNoSwitch: public CLeoBhWalk
{
  public:
    TYPEINFO("behavior/leo_walk", "Leo walking behavior without symmetrical switchers of observations")

    CLeoBhWalkNoSwi//    CLeoBhWalk *bh_;
tch() {}
    CLeoBhWalkNoSwitch(ISTGActuation *actuationInterface) : CLeoBhWalk(actuationInterface) {}
    void parseLeoState(const CLeoState &leoState, Vector &obs);
    void parseLeoAction(const Vector &action, Vector &target_action);
};
*/
/// Simulation of original Leo robot by Erik Schuitema.
class LeoWalkEnvironment: public LeoBaseEnvironment
{
  public:
    TYPEINFO("environment/leo_walk", "Leo walking environment")

  public:
    LeoWalkEnvironment();
    ~LeoWalkEnvironment() {}

    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    
    // From Environment
    virtual LeoWalkEnvironment *clone() const;
    virtual void start(int test, Vector *obs);
    virtual double step(const Vector &action, Vector *obs, double *reward, int *terminal);

    virtual void report(std::ostream &os) const;
    
  protected:
    int requested_action_dims_;
    Signal *mirror_sig_;
};

}

#endif /* GRL_LEO_WALK_ENVIRONMENT_H_ */
