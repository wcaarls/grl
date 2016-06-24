#ifndef GRL_LEO_ENVIRONMENT_H_
#define GRL_LEO_ENVIRONMENT_H_

#include <grl/environments/odesim/environment.h>
#include <LeoBhWalkSym.h>
#include <STGLeo.h>
#include <STGLeoSim.h>
#include <ThirdOrderButterworth.h>

namespace grl
{

struct ObserverStruct
{
  std::vector<int> angles;
  std::vector<int> angle_rates;
  std::vector<std::string> augmented;
};

// Base classes for Leo
class CLeoBhBase: public CLeoBhWalkSym
{
  public:
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
    enum LeaObservationWalk
    {
      owTorsoAngle,
      owTorsoAngleRate,
      owHipStanceAngle,
      owHipStanceAngleRate,
      owHipSwingAngle,
      owHipSwingAngleRate,
      owKneeStanceAngle,
      owKneeStanceAngleRate,
      owKneeSwingAngle,
      owKneeSwingAngleRate,
      owNumDims
    };

  public:
    CLeoBhBase(ISTGActuation *actuationInterface) : CLeoBhWalkSym(actuationInterface) {}

    int getHipStance()   {return mHipStance;}
    int getHipSwing()    {return mHipSwing;}
    int getKneeStance()  {return mKneeStance;}
    int getKneeSwing()   {return mKneeSwing;}
    int getAnkleStance() {return mAnkleStance;}
    int getAnkleSwing()  {return mAnkleSwing;}
    bool stanceLegLeft() {return mLastStancelegWasLeft;}

  public:
    void resetState();
    void setObserverStruct(ObserverStruct observer_struct) { observer_struct_ = observer_struct; }
    const ObserverStruct &getObserverStruct() { return observer_struct_; }
    void fillLeoState(const Vector &obs, const Vector &action, CLeoState &leoState);
    void parseLeoState(const CLeoState &leoState, Vector &obs);
    void updateDerivedStateVars(CLeoState *currentSTGState);
    void setCurrentSTGState(CLeoState *leoState);
    void setPreviousSTGState(CLeoState *leoState);
    void grlAutoActuateAnkles(Vector &out)
    {
      CSTGLeoSim *leoSim = dynamic_cast<CSTGLeoSim*>(mActuationInterface);
      CLeoBhWalkSym::autoActuateAnkles_FixedPos(leoSim);
      out.resize(2);
      out << leoSim->getJointVoltage(ljAnkleRight), leoSim->getJointVoltage(ljAnkleLeft);
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

    std::string jointIndexToName(int jointIndex) const;
    int jointNameToIndex(const std::string jointName) const;

  protected:
    CButterworthFilter<1>	mJointSpeedFilter[ljNumJoints];
    ObserverStruct observer_struct_;
};

/// Base class for simulated and real Leo
class LeoBaseEnvironment: public Environment
{
  public:
    LeoBaseEnvironment();
    ~LeoBaseEnvironment() {}

  protected:
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Environment
    virtual LeoBaseEnvironment *clone() const;
    virtual void start(int test);
    virtual void step(double tau, double reward, int terminal);
    virtual void report(std::ostream &os);

    // Own
    void set_bh(CLeoBhBase *bh) { bh_ = bh; }
    
  protected:
    CSTGLeoSim leoSim_;
    CLeoState leoState_;
    Environment *target_env_;
    std::string xml_;

    int observation_dims_, action_dims_;
    Vector observation_min_, observation_max_, action_min_, action_max_;
    int target_observation_dims_, target_action_dims_;
    Vector target_obs_, target_action_;

    // Exporter
    Exporter *exporter_;
    int test_;
    double time_test_, time_learn_, time0_;

  protected:
    void fillObserverStruct(const std::vector<std::string> &observed_names, ObserverStruct &observer_idx) const;
    int findVarIdx(const std::vector<std::string> &genericStates, std::string query) const;
    void configParseObservations(Configuration &config, const std::vector<std::string> &sensors);
    void configParseActions(Configuration &config, const std::vector<std::string> &actuators);

    void fillObserve(const std::vector<std::string> &genericStates,
                     const std::vector<std::string> &observeList,
                     std::vector<std::string> &observe) const;

    void fillActuate(const std::vector<std::string> &genericAction,
                     const std::vector<std::string> &actuateList,
                     Vector &out,
                     const std::string *req = NULL,
                     std::vector<int>  *reqIdx = NULL) const;

  private:
    CLeoBhBase *bh_; // makes it invisible in derived classes
};

}

#endif /* GRL_LEO_ENVIRONMENT_H_ */
