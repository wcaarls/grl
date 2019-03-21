#ifndef GRL_LEOSIM_ENVIRONMENT_H_
#define GRL_LEOSIM_ENVIRONMENT_H_

#include <grl/environments/odesim/environment.h>
#include "LeoBhWalkSym.h"
#include "STGLeo.h"
#include "STGLeoSim.h"
#include "ThirdOrderButterworth.h"

namespace grl
{
// Indices for ODEEnvironment (order of XML)
enum LeoStateVar
{
  svTorsoAngle,
  svTorsoAngleRate,
  svLeftArmAngle,
  svLeftArmAngleRate,
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
  svRightToeContact,
  svRightHeelContact,
  svLeftToeContact,
  svLeftHeelContact,
  
  // Augmented states for symmetry
  svStanceHipAngle,
  svStanceHipAngleRate,
  svSwingHipAngle,
  svSwingHipAngleRate,
  svStanceKneeAngle,
  svStanceKneeAngleRate,
  svSwingKneeAngle,
  svSwingKneeAngleRate,
  svStanceAnkleAngle,
  svStanceAnkleAngleRate,
  svSwingAnkleAngle,
  svSwingAnkleAngleRate,
  svStanceToeContact,
  svStanceHeelContact,
  svSwingToeContact,
  svSwingHeelContact,
  
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
  
  // Augmented actions for symmetry
  avStanceHipTorque,
  avSwingHipTorque,
  avStanceKneeTorque,
  avSwingKneeTorque,
  avStanceAnkleTorque,
  avSwingAnkleTorque,
  
  avNumActions
};

class CGrlLeoBhWalkSym : public CLeoBhWalkSym
{
  public:
    CGrlLeoBhWalkSym(ISTGActuation *actuationInterface) : CLeoBhWalkSym(actuationInterface) {}

    int getHipStance()   {return mHipStance;}
    int getHipSwing()    {return mHipSwing;}
    int getKneeStance()  {return mKneeStance;}
    int getKneeSwing()   {return mKneeSwing;}
    int getAnkleStance() {return mAnkleStance;}
    int getAnkleSwing()  {return mAnkleSwing;}
    bool stanceLegLeft() {return mLastStancelegWasLeft;}

  public:
    void resetState();
    void fillLeoState(const Vector &obs, const Vector &action, CLeoState *leoState);
    void parseLeoState(const CLeoState &leoState, Vector *obs);
    void updateDerivedStateVars(CLeoState *currentSTGState);
    void setCurrentSTGState(CLeoState *leoState);
    void setPreviousSTGState(CLeoState *leoState);
    void grlAutoActuateAnkles(Vector *out)
    {
      CSTGLeoSim *leoSim = dynamic_cast<CSTGLeoSim*>(mActuationInterface);
      CLeoBhWalkSym::autoActuateAnkles_FixedPos(leoSim);
      out->resize(2);
      *out << leoSim->getJointVoltage(ljAnkleRight), leoSim->getJointVoltage(ljAnkleLeft);
    }
    double grlAutoActuateArm()
    {
      CSTGLeoSim *leoSim = dynamic_cast<CSTGLeoSim*>(mActuationInterface);
      CLeoBhWalkSym::autoActuateArm(leoSim);
      return leoSim->getJointVoltage(ljShoulder);
    }
    double grlAutoActuateKnee()
    {
      CSTGLeoSim *leoSim = dynamic_cast<CSTGLeoSim*>(mActuationInterface);
      CLeoBhWalkSym::autoActuateKnees(leoSim);
      return stanceLegLeft() ? leoSim->getJointVoltage(ljKneeLeft) : leoSim->getJointVoltage(ljKneeRight);
    }

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

    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);
    
    // From ODEEnvironment
    virtual LeoSimEnvironment *clone();
    
    // From Environment
    virtual void start(int test, Observation *obs);
    virtual double step(const Action &action, Observation *obs, double *reward, int *terminal);
    virtual void report(std::ostream &os) const;
    
  protected:
    CSTGLeoSim leoSim_;
    CLeoState leoState_;
    CGrlLeoBhWalkSym bhWalk_;

    // Exporter
    Exporter *exporter_;
    int test_;
    double time_test_, time_learn_, time0_;

  private:
    void fillObserve(const std::vector<std::string> &observeList,
                     IndexVector *out) const;

    void fillActuate(const std::vector<std::string> &actuateList,
                     IndexVector *out) const;
  private:
    Observation ode_obs_;
    Action ode_action_;
    IndexVector observe_, actuate_;
};

}

#endif /* GRL_LEOSIM_ENVIRONMENT_H_ */
