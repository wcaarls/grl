/*
 * LeoBhWalkSym.h
 *
 * Reinforcement Learning walking behavior / policy EXPLOITING STANCE/SWING SYMMETRY
 *
 *  Created on: Sep 1, 2009
 *      Author: Erik Schuitema
 */

#ifndef LEOBHWALKSYM_H_
#define LEOBHWALKSYM_H_

#include <STGAgentQLeo.h>

#define LEOBHWALKSYM_MAX_NUM_ACTIONS  3

// Leo RL walking behavior
class CLeoBhWalkSym: public CSTGAgentQLeo
{
  protected:
    double        mTotalStepTime;

    // Learning algorithm options
    bool          mUseEffectiveAction;        // Calculate the effectively executed action to improve convergence
    bool          mGeneralizeActions;         // Generalize between actions in the function approximator
    bool          mIsObserving;

    // Reward variables
    double        mRwTime;                    // Time penalty
    double        mRwFootstepDist;            // Footstep reward per meter
    double        mRwFootstepDistCont;        // Continuous reward for moving swing foot in front of the stance foot
    double        mRwFootstepMaxLength;       // Maximum length of a footstep in order to get reward for it
    double        mRwFootstepBackward;        // Penalty per footstep that has negative length
    double        mRwEnergy;                  // Energy reward per Watt
    double        mRwFootClearance;           // Reward for distance between the floor and the lowest point of the stance leg
    double        mRwFootClearanceThreshold;  // Foot clearance threshold above which no reward will be given
    double        mRwHipAngleChange;          // Reward for a change in hip angle - can possibly be used to stimulate leg swinging
    double        mRwDoomedToFall;            // Reward for situations from which the robot is doomed to fall
    double        mRwTorsoUpright;            // Reward for difference between torso angle and desired torso angle (~0.0)
    double        mRwTorsoUprightAngle;       // Desired angle for mRwTorsoUpright
    double        mRwTorsoUprightAngleMargin; // Desired angle margin for mRwTorsoUpright, defined as angle deviation for which reward is 50% of its max.
    double        mRwDoubleStance;            // Reward for both feet touching the floor

    // State space scaling variables
    double        mScaleFactTorsoAngle;
    double        mScaleFactTorsoAngleRate;
    double        mScaleFactHipStanceAngle;
    double        mScaleFactHipStanceAngleRate;
    double        mScaleFactHipSwingAngle;
    double        mScaleFactHipSwingAngleRate;
    double        mScaleFactKneeStanceAngle;
    double        mScaleFactKneeStanceAngleRate;
    double        mScaleFactKneeSwingAngle;
    double        mScaleFactKneeSwingAngleRate;
    double        mMultiResScaleFact;         // If > 0, multi-resolution tile coding will be used with an additional layer, uniformly scaled with this factor (state space only, not action space)
    bool          mContinueAfterFall;

    // Action space settings
//    int           mNumActionsPerJoint;
//    double        mScaleFactVoltage;

    // Termination variables
    uint64_t      mTrialTimeout;              // Trial timeout time in microseconds

    // Settings for the pre-programmed parts of the walking controller (for the observable controller, more parts are preprogrammed than for learning)
    double        mPreProgTorsoAngle;
    double        mPreProgHipAngle;
    double        mPreProgShoulderAngle;
    double        mPreProgAnkleAngle;
    double        mPreProgStanceKneeAngle;
    uint64_t      mPreProgEarlySwingTime;
    double        mPreProgExploreRate;


    double        mDesiredFrequency;          // Desired frequency for this policy
    uint32_t      mDesiredMemorySize;         // Desired memory size
    uint32_t      mDesiredNumTilings;         // Desired number of tilings

    // Derived state variables
    int           mHipStance, mHipSwing, mKneeStance, mKneeSwing, mAnkleStance, mAnkleSwing;
    bool          mStanceFootContact, mSwingFootContact;
    int           mLastStancelegWasLeft;      // -1 means undefined
    int           mLastRewardedFoot;
    bool          mMadeFootstep;
    double        mFootstepLength;
    double        mLastFootstepLength;
    double        mFootClearance;             // Distance of a swing leg to a floor
    double        mLeftAnklePos, mRightAnklePos;
    uint64_t      mSwingTime;                 // Time since last footstep [us]
    int           mFootContactNum;

    // Performance variables
    int           mNumFootsteps;              // Number of footsteps since last reset
    int           mNumFalls;                  // Number of times the walker fell
    double        mWalkedDistance;            // The accumulation of all footstep lengths since last reset
    double        mTrialEnergy;               // The accumulation of all energy used since last reset

    void          updateDerivedStateVars(CLeoState* currentSTGState);
    void          autoActuateKnees(ISTGActuation* actuationInterface);
    void          autoActuateAnkles_FixedPos(ISTGActuation* actuationInterface);
    void          autoActuateArm(ISTGActuation* actuationInterface);
    inline double clip(double value, double min, double max)
    {
      return std::max(min, std::min(value, max));
    }

    // Partial reward functions
    double        getFootstepReward();
    double        getEnergyUsage();
    double        getJointMotorWork(int jointIndex);

    // This first period will be used to observe a pre-programmed controller
    uint64_t      mObservingTime;

  public:
    CLeoBhWalkSym(ISTGActuation *actuationInterface);

    bool                  readConfig(const CConfigSection &xmlRoot);
    virtual double        calculateReward();
    virtual bool          isDoomedToFall(CLeoState* state, bool report);
    virtual std::string   getProgressReport(double trialTime);
};

#endif /* LEOBHWALKSYM_H_ */
