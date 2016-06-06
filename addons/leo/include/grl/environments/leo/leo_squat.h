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
  #define siDirection (siKneeSwingAngleRate+1)

  public:
    CLeoBhSquat(ISTGActuation *actuationInterface) : CLeoBhBase(actuationInterface), direction_(-1), prev_direction_(-1) {}
    double calculateReward();
    void parseLeoState(const CLeoState &leoState, Vector &obs);
    void setDirection(int direction) { direction = direction_; }

  protected:
    int direction_, prev_direction_;
    bool isSitting(const double *x) const;
    bool isStanding(const double *x) const;
    double r(const double *x, const double* a) const;
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
    CLeoBhSquat *bh_;
};

}

#endif /* GRL_LEO_SQUAT_ENVIRONMENT_H_ */
