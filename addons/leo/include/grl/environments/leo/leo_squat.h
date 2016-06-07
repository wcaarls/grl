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
  enum LeaObservationSquat
  {
    osTorsoAngle,
    osTorsoAngleRate,
    osHipStanceAngle,
    osHipStanceAngleRate,
    osKneeStanceAngle,
    osKneeStanceAngleRate,
//    osDirection,
    osNumDims
  };

  public:
    CLeoBhSquat(ISTGActuation *actuationInterface) : CLeoBhBase(actuationInterface), direction_(-1), prev_direction_(-1) {}
    void resetState();
    double calculateReward();
    void parseLeoState(const CLeoState &leoState, Vector &obs);
    void setDirection(int direction) { direction = direction_; }
    bool isDoomedToFall(CLeoState* state, bool report);

  protected:
    int direction_, prev_direction_;
    double pHipHeight_, pHipPos_, cHipHeight_, cHipPos_;
    bool isSitting() const;
    bool isStanding() const;
    void getHipHeight(const double *x, double &hipHeight, double &hipPos) const;
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
