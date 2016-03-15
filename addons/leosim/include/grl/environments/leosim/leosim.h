#ifndef GRL_LEOSIM_ENVIRONMENT_H_
#define GRL_LEOSIM_ENVIRONMENT_H_

#include <grl/environments/odesim/environment.h>
#include "LeoBhWalkSym.h"
#include "STGLeo.h"
#include "STGLeoSim.h"
#include "ThirdOrderButterworth.h"

namespace grl
{

class CGrlLeoBhWalkSym : public CLeoBhWalkSym
{
  public:
    enum LeoStateVar
    {
      svTorsoAngle,
      svTorsoAngleRate,
      svShoulderAngle,
      svShoulderAngleRate,
      svRightHipAngle,
      svRightHipAngleRate,
      svLeftHipAngle,
      svLeftHipAngleRate,
      svRightKneeAngle,
      svRightKneeAngleRate,
      svLeftKneeAngle,
      svLeftKneeAngleRate,
      svRightAnkleAngle,
      svRightAnkleAngleRate,
      svLeftAnkleAngle,
      svLeftAnkleAngleRate,
      svNumStates
    };
    enum LeoActionVar
    {
      avLeftArmTorque,
      avRightHipTorque,
      avLeftHipTorque,
      avRightKneeTorque,
      avLeftKneeTorque,
      avRightAnkleTorque,
      avLeftAnkleTorque,
      svNumActions
    };

  public:
    CGrlLeoBhWalkSym(ISTGActuation *actuationInterface) : CLeoBhWalkSym(actuationInterface) {init();}
    ~CGrlLeoBhWalkSym() {}

    int getHipStance()   {return mHipStance;}
    int getHipSwing()    {return mHipSwing;}
    int getKneeStance()  {return mKneeStance;}
    int getKneeSwing()   {return mKneeSwing;}
    int getAnkleStance() {return mAnkleStance;}
    int getAnkleSwing()  {return mAnkleSwing;}
    bool getStanceFootContact() {return mStanceFootContact;}
    bool getSwingFootContact()  {return mSwingFootContact;}

  public:
    void resetState();
    void parseOdeObs(const Vector &obs, CLeoState &leoState);
    void parseLeoState(const CLeoState &leoState, Vector &obs);
    void updateDerivedStateVars(CLeoState *currentSTGState);
    void setCurrentSTGState(CLeoState *leoState);
    void setPreviousSTGState(CLeoState *leoState);
    void grlAutoActuateAnkles(Vector &out)
    {
      CSTGLeoSim* aI = dynamic_cast<CSTGLeoSim*>(mActuationInterface);
      CLeoBhWalkSym::autoActuateAnkles_FixedPos(aI);
      out.resize(2);
      out << aI->getJointVoltage(mAnkleStance), aI->getJointVoltage(mAnkleSwing);
    }
    void grlAutoActuateArm(Vector &out)
    {
      CSTGLeoSim* aI = dynamic_cast<CSTGLeoSim*>(mActuationInterface);
      CLeoBhWalkSym::autoActuateArm(aI);
      out.resize(1);
      out << aI->getJointVoltage(ljShoulder);
    }

  protected:
    void init();

  protected:
    CButterworthFilter<1>	mJointSpeedFilter[ljNumJoints];
};

/// Simulation of original Leo robot by Erik Schuitema.
class LeoSimEnvironment : public ODEEnvironment
{
  public:
    TYPEINFO("environment/leosim", "Leo simulation environment")

  public:
    LeoSimEnvironment();
    ~LeoSimEnvironment() {}

    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);
    
    // From ODEEnvironment
    virtual LeoSimEnvironment *clone();
    
    // From Environment
    virtual void start(int test, Vector *obs);
    virtual double step(const Vector &action, Vector *obs, double *reward, int *terminal);
    
  protected:
    CLeoState leoState_;
    CGrlLeoBhWalkSym bhWalk_;
    int observation_dims_, action_dims_;
    int ode_observation_dims_, ode_action_dims_;
    int learn_stance_knee_;

  private:
    void fillObserve(const std::vector<CGenericStateVar> &genericStates,
                     const std::vector<std::string> &observeList,
                     Vector &out);

    void fillActuate(const std::vector<CGenericActionVar> &genericAction,
                     const std::vector<std::string> &actuateList,
                     Vector &out);
  private:
    Vector ode_obs_;
    Vector observe_, actuate_;
    Vector ode_action_;
};

}

#endif /* GRL_LEOSIM_ENVIRONMENT_H_ */
