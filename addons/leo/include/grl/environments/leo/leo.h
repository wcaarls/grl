#ifndef GRL_LEO_ENVIRONMENT_H_
#define GRL_LEO_ENVIRONMENT_H_

#include <grl/environments/odesim/environment.h>
#include <LeoBhWalkSym.h>
#include <STGLeo.h>
#include <STGLeoSim.h>
#include <ThirdOrderButterworth.h>
#include <grl/signal.h>

namespace grl
{
struct TargetInterface
{
  struct ObserverInterface
  {
    std::vector<int> angles;
    std::vector<int> angle_rates;
    std::vector<std::string> augmented;
  };
  struct ActuatorInterface
  {
    std::vector<int> voltage;               // mapping: target_action = agent_action(actions == i)
    std::vector<std::string> autoActuated;  // textual names of actions that should be automatically actuated, for them 'actions = -1'
  };

  ObserverInterface observer;     // used for stance leg being left
  ObserverInterface observer_sym; // used for stance leg being right
  ActuatorInterface actuator;     // used for stance leg being left
  ActuatorInterface actuator_sym; // used for stance leg being right
};

enum LeoSignalType {lstNone, lstContact}; // Leo has the additional to default signal type

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
    enum LeoObservationWalk
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
    CLeoBhBase() : CLeoBhWalkSym(&leoSim_) {}

    int getHipStance()   {return mHipStance;}
    int getHipSwing()    {return mHipSwing;}
    int getKneeStance()  {return mKneeStance;}
    int getKneeSwing()   {return mKneeSwing;}
    int getAnkleStance() {return mAnkleStance;}
    int getAnkleSwing()  {return mAnkleSwing;}
    bool stanceLegLeft() {return mLastStancelegWasLeft;}

  public:
    virtual void resetState(double time0);
    virtual void parseLeoState(const CLeoState &leoState, Vector &obs);
    virtual void parseLeoAction(const Vector &action, Vector &target_action) = 0;

    void setObserverInterface(const TargetInterface::ObserverInterface oi, const TargetInterface::ObserverInterface oi_sym) { interface_.observer = oi; interface_.observer_sym = oi_sym; }
    void setActuatorInterface(const TargetInterface::ActuatorInterface ai, const TargetInterface::ActuatorInterface ai_sym) { interface_.actuator = ai; interface_.actuator_sym = ai_sym; }
    const TargetInterface &getInterface() const { return interface_; }

    void fillLeoState(const Vector &obs, const Vector &action, CLeoState &leoState);
    void updateDerivedStateVars(CLeoState *currentSTGState);
    bool madeFootstep();
    void setCurrentSTGState(CLeoState *leoState);
    void setPreviousSTGState(CLeoState *leoState);
    void grlAutoActuateRightAnkle(double &actionRightAnkle)
    {
      CSTGLeoSim *leoSim = dynamic_cast<CSTGLeoSim*>(mActuationInterface);
      CLeoBhWalkSym::autoActuateAnkles_FixedPos(leoSim);
      actionRightAnkle = leoSim->getJointVoltage(ljAnkleRight);
    }
    void grlAutoActuateLeftAnkle(double &actionLeftAnkle)
    {
      CSTGLeoSim *leoSim = dynamic_cast<CSTGLeoSim*>(mActuationInterface);
      CLeoBhWalkSym::autoActuateAnkles_FixedPos(leoSim);
      actionLeftAnkle = leoSim->getJointVoltage(ljAnkleLeft);
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
    TargetInterface interface_;

  private:
    CSTGLeoSim leoSim_;
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
    virtual void start(int test);
    virtual void step(double tau, double reward, int terminal);
    virtual void report(std::ostream &os) const;
    
  protected:
    CLeoState leoState_;
    Environment *target_env_;
    std::string xml_;
    CLeoBhBase *bh_;

    int observation_dims_, action_dims_;
    Vector target_obs_, target_action_;

    // Exporter
    Exporter *exporter_;
    int test_;
    double time_test_, time_learn_, time0_;

  protected:
    int findVarIdx(const std::vector<CGenericStateVar> &genericStates, std::string query) const;

    // Observations
    void configParseObservations(Configuration &config, const std::vector<CGenericStateVar> &sensors);
    void fillObserve(const std::vector<CGenericStateVar> &genericStates,
                     const std::vector<std::string> &observeList,
                     std::vector<std::string> &observed) const;
    void fillObserver(const std::vector<std::string> &observed, TargetInterface::ObserverInterface &observer_interface) const;

    // Actions
    void configParseActions(Configuration &config, const std::vector<CGenericActionVar> &actuators);
    void fillActuate(const std::vector<CGenericActionVar> &genericAction,
                     const std::vector<std::string> &actuateList,
                     TargetInterface::ActuatorInterface &int_actuator) const;

  private:
    VectorSignal *sub_transition_type_;
};

}

#endif /* GRL_LEO_ENVIRONMENT_H_ */
