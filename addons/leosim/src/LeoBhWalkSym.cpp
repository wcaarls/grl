/*
 * LeoBhWalk.cpp
 *
 *  Created on: Sep 1, 2009
 *      Author: Erik Schuitema
 */

#include "LeoBhWalkSym.h"
//#include <QSpaceTileCoding.h>
#include <cmath>

#ifndef clip
#define clip(x, minval, maxval) (std::max(std::min(x, maxval), minval))
#endif

CLeoBhWalkSym::CLeoBhWalkSym(ISTGActuation *actuationInterface):
  CSTGAgentQLeo(actuationInterface),
  mPreProgTorsoAngle(-0.09),
  mPreProgHipAngle(0.680),
  mPreProgShoulderAngle(-15.0*M_PI/180.0),
  mPreProgAnkleAngle(0.0),
  mPreProgStanceKneeAngle(0.0),
  mPreProgEarlySwingTime(0.184E6),
  mPreProgExploreRate(0.05),
  mHipStance(ljHipLeft),
  mHipSwing(ljHipRight),
  mKneeStance(ljKneeLeft),
  mKneeSwing(ljKneeRight),
  mAnkleStance(ljAnkleLeft),
  mAnkleSwing(ljAnkleRight),
  mStanceFootContact(true),
  mSwingFootContact(false),
  mPrintState(false)
{
  mDesiredFrequency      = 30.0;
  mDesiredMemorySize      = 1024*1024*4;
  mDesiredNumTilings      = 16;
//  mAlgo            = laSARSA;
  mUseEffectiveAction      = false;
  mRwTime            = -1.0;
  mRwFootstepDist        = 500;
  mRwFootstepDistCont      = 0.0;
  mRwFootstepMaxLength    = 100.0;
  mRwFootstepBackward      = 0.0;
  mRwEnergy          = 0.0;
  mRwFootClearance      = 0.0;
  mRwFootClearanceThreshold  = 0.0;
  mRwDoomedToFall        = 0.0;
  mRwHipAngleChange      = 0.0;
  mRwTorsoUpright        = 0.0;
  mRwTorsoUprightAngle    = -0.1;
  mRwTorsoUprightAngleMargin  = 0.1;
  mRwDoubleStance        = 0.0;
  mTrialTimeout        = (uint64_t)15E6;
  mLastStancelegWasLeft    = 0;
  mLastRewardedFoot      = lpFootLeft;
  mNumFootsteps        = 0;
  mNumFalls          = 0;
  mWalkedDistance        = 0.0;
  mTrialEnergy        = 0.0;
  mObservingTime         = (uint64_t)200E6;
  mLearnSwingKnee        = true;
  mGeneralizeActions      = true;
  mSwingTime          = 0;


  mScaleFactTorsoAngle      = 5.0;
  mScaleFactTorsoAngleRate    = 0.25;
  mScaleFactHipStanceAngle    = 2.64;
  mScaleFactHipStanceAngleRate  = 0.25;
  mScaleFactHipSwingAngle      = 2.31;
  mScaleFactHipSwingAngleRate    = 0.11;
  mScaleFactKneeStanceAngle    = 0.66;
  mScaleFactKneeStanceAngleRate  = 0.15;
  mScaleFactKneeSwingAngle    = 1.33;
  mScaleFactKneeSwingAngleRate  = 0.15;
  mMultiResScaleFact        = -1.0;

  mNumActionsPerJoint        = 7;
  mScaleFactVoltage        = 0.2;

  // Set mPreviousAction to 0.0 - this is the initialization value of the torques in the simulator, and probably close to the real robot's situation (although it doesn't matter much)
  for (int iAction=0; iAction<LEOBHWALKSYM_MAX_NUM_ACTIONS; iAction++)
    mPreviousAction[iAction] = 0.0;

//  setAlgo(mAlgo);
  // Settings for 1.0/50.0 step time:
  //setLearnParameters(0.25, 0.05, 0.9908, 0.9250);
  // Settings for 1.0/75.0 step time:
  //setLearnParameters(0.25, 0.05, 0.9939, 0.9494);

  // setLearnParameters(0.20, 0.05, 0.9990, 0.9787); NOW DONE IN XML!
}


bool CLeoBhWalkSym::readConfig(const CConfigSection &xmlRoot)
{
  CConfigSection configNode = xmlRoot.section("policy");

  bool configresult = true;
  configresult &= mLogAssert(configNode.get("rewardTime", &mRwTime));
  configresult &= mLogAssert(configNode.get("rewardFootstepDist", &mRwFootstepDist));
  configresult &= mLogAssert(configNode.get("rewardFootstepDistCont", &mRwFootstepDistCont));
  configresult &= mLogAssert(configNode.get("rewardFootstepMaxLength", &mRwFootstepMaxLength));
  configresult &= mLogAssert(configNode.get("rewardFootstepBackward", &mRwFootstepBackward));
  configresult &= mLogAssert(configNode.get("rewardEnergy", &mRwEnergy));
  configresult &= mLogAssert(configNode.get("rewardFootClearance", &mRwFootClearance));
  configresult &= mLogAssert(configNode.get("rewardFootClearanceThreshold", &mRwFootClearanceThreshold));
  configresult &= mLogAssert(configNode.get("rewardDoomedToFall", &mRwDoomedToFall));
  configresult &= mLogAssert(configNode.get("rewardHipAngleChange", &mRwHipAngleChange));
  configresult &= mLogAssert(configNode.get("rewardDoubleStance", &mRwDoubleStance));
  configresult &= mLogAssert(configNode.get("rewardTorsoUpright", &mRwTorsoUpright));
  configresult &= mLogAssert(configNode.get("rewardTorsoUprightAngle", &mRwTorsoUprightAngle));
  configresult &= mLogAssert(configNode.get("rewardTorsoUprightAngleMargin", &mRwTorsoUprightAngleMargin));

  double timeSeconds=0;
  configresult &= mLogAssert(configNode.get("trialTimeoutSeconds", &timeSeconds));
  mTrialTimeout = (uint64_t)(timeSeconds*1E6);
  configresult &= mLogAssert(configNode.get("observingTimeSeconds", &timeSeconds));
  mObservingTime = (uint64_t)(timeSeconds*1E6);
  configresult &= mLogAssert(configNode.get("learnSwingKnee", &mLearnSwingKnee));

  configNode = xmlRoot.section("ode");
  configNode.get("steptime", &mTotalStepTime);

  return configresult;
}

/*
bool CLeoBhWalkSym::init()
{
  enableProgressReporting(true);

  // Create Q-space
  CQSpaceTileCoding *qspace = new CQSpaceTileCoding;  // Will be destroyed in ~CAgentQ()
  setQSpace(qspace);
  qspace->setNumTilings(mDesiredNumTilings);
  qspace->setMemorySize(mDesiredMemorySize);
  qspace->generalizeActions(mGeneralizeActions);

  mAgentState.setNumStateVars(siNumStateDims);
  mAgentState.enableWrapping(false);
  if (mLearnSwingKnee)
    mAgentAction.setNumActionVars(3);
  else
    mAgentAction.setNumActionVars(2);
  double maxVoltage = getActuationInterface()->getJointMaxVoltage(ljHipLeft);
  mLogInfoLn("Max. allowed voltage calculated from temperature compensation model: " << maxVoltage);
  // Action 0 is hipStance, action 1 is hipSwing, action 2 is kneeSwing
  // Gaat prima op 75Hz:
  //mAgentAction.mDescriptors[0].set(-maxVoltage, maxVoltage, 5, 0.2);
  //mAgentAction.mDescriptors[1].set(-maxVoltage, maxVoltage, 5, 0.2);
  //mAgentAction.mDescriptors[0].set(-maxVoltage, maxVoltage, 21, 0.75);
  //mAgentAction.mDescriptors[1].set(-maxVoltage, maxVoltage, 21, 0.75);
  // Probeersel:
  mAgentAction.mDescriptors[0].set(-maxVoltage, maxVoltage, mNumActionsPerJoint, mScaleFactVoltage);
  mAgentAction.mDescriptors[1].set(-maxVoltage, maxVoltage, mNumActionsPerJoint, mScaleFactVoltage);
  if (mLearnSwingKnee)
    mAgentAction.mDescriptors[2].set(-maxVoltage, maxVoltage, mNumActionsPerJoint, mScaleFactVoltage);

  mShowQValues = false;

  if (mUseEffectiveAction)
  {
    // We are going to make use of the effective action mechanism,
    mLogNoticeLn("Using effective action updating, taking delay into account");
    // so copy the descriptors from mAgentAction to mAgentEffectiveAction
    mAgentEffectiveAction = mAgentAction;
  }

  bool result = CSTGAgentQLeo::init();
  displayVariables();
  return result;
}
*/
/*
bool CLeoBhWalkSym::init(std::ifstream &fileInStream)
{
  enableProgressReporting(true);

  // Create Q-space
  CQSpaceTileCoding *qspace = new CQSpaceTileCoding;  // Will be destroyed in ~CAgentQ()
  setQSpace(qspace);

  // mAgentState and mAgentAction are being read by CAgentQ::init(..)
  bool result = true;
  result &= CSTGAgentQLeo::init(fileInStream);

  // Copy effective action settings
  if (mUseEffectiveAction)
  {
    // We are going to make use of the effective action mechanism,
    mLogNoticeLn("Using effective action updating, taking delay into account");
    // so copy the descriptors from mAgentAction to mAgentEffectiveAction
    mAgentEffectiveAction = mAgentAction;
  }

  // Display agent variables again after loading
  mLogInfoLn("Agent variables after loading policy:");
  displayVariables();
  return result;

}
*/

/*
bool CLeoBhWalkSym::save(std::ofstream& fileOutStream)
{
  bool result = true;
  result &= CSTGAgentQLeo::save(fileOutStream);
  return result;
}
*/
/*
void CLeoBhWalkSym::updateState(CLeoState* currentSTGState)
{
  // Update derived state variables that make calculations easier
  updateDerivedStateVars(currentSTGState);

  // Fill mAgentState
*/
  /*
  // For 30 Hz, works well
  mAgentState.setStateVar(siTorsoAngle,      7.5*currentSTGState->mJointAngles[ljTorso]);
  mAgentState.setStateVar(siTorsoAngleRate,    0.125*currentSTGState->mJointSpeeds[ljTorso]);
  mAgentState.setStateVar(siHipStanceAngle,    4.0*currentSTGState->mJointAngles[mHipStance]);
  mAgentState.setStateVar(siHipStanceAngleRate,  0.125*currentSTGState->mJointSpeeds[mHipStance]);
  mAgentState.setStateVar(siHipSwingAngle,    3.5*currentSTGState->mJointAngles[mHipSwing]);
  mAgentState.setStateVar(siHipSwingAngleRate,  0.055*currentSTGState->mJointSpeeds[mHipSwing]);
  mAgentState.setStateVar(siKneeStanceAngle,    1.0*currentSTGState->mJointAngles[mKneeStance]);
  mAgentState.setStateVar(siKneeStanceAngleRate,  0.075*currentSTGState->mJointSpeeds[mKneeStance]);
  mAgentState.setStateVar(siKneeSwingAngle,    2.0*currentSTGState->mJointAngles[mKneeSwing]);
  if (mLearnSwingKnee)
    mAgentState.setStateVar(siKneeSwingAngleRate,  0.075*currentSTGState->mJointSpeeds[mKneeSwing]);
  else
    mAgentState.setStateVar(siKneeSwingAngleRate, 0);
    */
/*
  mAgentState.setStateVar(siTorsoAngle,      mScaleFactTorsoAngle      *currentSTGState->mJointAngles[ljTorso]);
  mAgentState.setStateVar(siTorsoAngleRate,    mScaleFactTorsoAngleRate    *currentSTGState->mJointSpeeds[ljTorso]);
  mAgentState.setStateVar(siHipStanceAngle,    mScaleFactHipStanceAngle    *currentSTGState->mJointAngles[mHipStance]);
  mAgentState.setStateVar(siHipStanceAngleRate,  mScaleFactHipStanceAngleRate  *currentSTGState->mJointSpeeds[mHipStance]);
  mAgentState.setStateVar(siHipSwingAngle,    mScaleFactHipSwingAngle      *currentSTGState->mJointAngles[mHipSwing]);
  mAgentState.setStateVar(siHipSwingAngleRate,  mScaleFactHipSwingAngleRate    *currentSTGState->mJointSpeeds[mHipSwing]);
  mAgentState.setStateVar(siKneeStanceAngle,    mScaleFactKneeStanceAngle    *currentSTGState->mJointAngles[mKneeStance]);
  mAgentState.setStateVar(siKneeStanceAngleRate,  mScaleFactKneeStanceAngleRate  *currentSTGState->mJointSpeeds[mKneeStance]);
  mAgentState.setStateVar(siKneeSwingAngle,    mScaleFactKneeSwingAngle    *currentSTGState->mJointAngles[mKneeSwing]);
  if (mLearnSwingKnee)
    mAgentState.setStateVar(siKneeSwingAngleRate,  mScaleFactKneeSwingAngleRate*currentSTGState->mJointSpeeds[mKneeSwing]);
  else
    mAgentState.setStateVar(siKneeSwingAngleRate, 0);

  if (mPrintState)
    mLogInfoLn("S: " << mAgentState.toStr() << ", StCont:" << (mStanceFootContact?"Y":"N") << ", SwCont:" << (mSwingFootContact?"Y":"N"));

  if (mIsObserving)
    setAlgo(laQObserve);
  else
    setAlgo(mAlgo);
}
*/
void CLeoBhWalkSym::updateDerivedStateVars(CLeoState* currentSTGState)
{
  // Backup last footstep length
  // TODO: move this line to a more logical place
  mLastFootstepLength = mFootstepLength;

  // Reset last footstep length when footstep is made
  if (mMadeFootstep)
    mLastFootstepLength = -mFootstepLength;

  // Determine foot contact
  bool leftFootContact      = (currentSTGState->mFootContacts & LEO_FOOTSENSOR_LEFT_HEEL) || (currentSTGState->mFootContacts & LEO_FOOTSENSOR_LEFT_TOE);
  bool rightFootContact     = (currentSTGState->mFootContacts & LEO_FOOTSENSOR_RIGHT_HEEL) || (currentSTGState->mFootContacts & LEO_FOOTSENSOR_RIGHT_TOE);
  // Determine foot position relative to the hip axis
  double upLegLength        = 0.116;  // length of the thigh
  double loLegLength        = 0.1045; // length of the shin
  double leftHipAbsAngle    = currentSTGState->mJointAngles[ljTorso] + currentSTGState->mJointAngles[ljHipLeft];
  double leftKneeAbsAngle   = leftHipAbsAngle + currentSTGState->mJointAngles[ljKneeLeft];
  double leftAnkleAbsAngle  = leftKneeAbsAngle + currentSTGState->mJointAngles[ljAnkleLeft];
  double rightHipAbsAngle   = currentSTGState->mJointAngles[ljTorso] + currentSTGState->mJointAngles[ljHipRight];
  double rightKneeAbsAngle  = rightHipAbsAngle + currentSTGState->mJointAngles[ljKneeRight];
  double rightAnkleAbsAngle = rightKneeAbsAngle + currentSTGState->mJointAngles[ljAnkleRight];
  double leftAnklePos       = upLegLength*sin(leftHipAbsAngle) + loLegLength*sin(leftKneeAbsAngle);   // in X direction (horizontal)
  double rightAnklePos      = upLegLength*sin(rightHipAbsAngle) + loLegLength*sin(rightKneeAbsAngle); // in X direction (horizontal)

  // Calculate the absolute positions of the toes and heels; assume that the lowest point touches the floor.
  // Start calculations from the hip. For convenience, take Z upwards as positive (some minus-signs are flipped).
  const double ankleHeelDX  = 0.0315;  //X = 0.009 - footlength/2 = −0.0315
  const double ankleHeelDZ  = 0.04859; //Z = -0.03559 - footwheelradius = -0.04859
  const double ankleToeDX   = -0.0495; //X = 0.009 + footlength/2 = 0.009 + 0.081/2 = 0.0495
  const double ankleToeDZ   = 0.04859; //Z = -0.03559 - footwheelradius = -0.03559 - 0.013 = -0.04859

  double leftAnkleZ         = upLegLength*cos(leftHipAbsAngle) + loLegLength*cos(leftKneeAbsAngle);
  double leftHeelZ          = leftAnkleZ + ankleHeelDZ*cos(leftAnkleAbsAngle) + ankleHeelDX*sin(leftAnkleAbsAngle);
  double leftToeZ           = leftAnkleZ + ankleToeDZ*cos(leftAnkleAbsAngle) + ankleToeDX*sin(leftAnkleAbsAngle);
  double rightAnkleZ        = upLegLength*cos(rightHipAbsAngle) + loLegLength*cos(rightKneeAbsAngle);
  double rightHeelZ         = rightAnkleZ + ankleHeelDZ*cos(rightAnkleAbsAngle) + ankleHeelDX*sin(rightAnkleAbsAngle);
  double rightToeZ          = rightAnkleZ + ankleToeDZ*cos(rightAnkleAbsAngle) + ankleToeDX*sin(rightAnkleAbsAngle);

  double hipHeight          = std::max(std::max(leftHeelZ, leftToeZ), std::max(rightHeelZ, rightToeZ));
  leftHeelZ  = hipHeight - leftHeelZ;
  leftToeZ   = hipHeight - leftToeZ;
  rightHeelZ = hipHeight - rightHeelZ;
  rightToeZ  = hipHeight - rightToeZ;
  //mLogInfoLn("leftHeelZ:" << leftHeelZ << ", leftToeZ:" << leftToeZ << ", rightHeelZ:" << rightHeelZ << ", rightToeZ:" << rightToeZ);

  bool leftIsStance;
  if (leftFootContact && (!rightFootContact))
    leftIsStance = true;
  else
  if ((!leftFootContact) && rightFootContact)
    leftIsStance = false;
  else
  if ((!leftFootContact) && (!rightFootContact))
    leftIsStance = (bool)mLastStancelegWasLeft;  // Actually, the stance leg is undetermined. Periods of flight should be short (or nonexistent) in practice.
  else
  // Both feet on the floor: the front leg is stance leg
  {
    if (rightAnklePos > leftAnklePos)
      leftIsStance = false;
    else
      leftIsStance = true;
  }

  // Determine the right joint indices and foot clearance
  if (leftIsStance)
  {
    mHipStance          = ljHipLeft;
    mKneeStance         = ljKneeLeft;
    mAnkleStance        = ljAnkleLeft;
    mStanceFootContact  = leftFootContact;
    mHipSwing           = ljHipRight;
    mKneeSwing          = ljKneeRight;
    mAnkleSwing         = ljAnkleRight;
    mSwingFootContact   = rightFootContact;
    // Calculate clearance for right foot
    mFootClearance      = std::min(rightToeZ, rightHeelZ);
    //mLogInfoLn("Foot clearance is " << mFootClearance);
  }
  else
  {
    mHipStance          = ljHipRight;
    mKneeStance         = ljKneeRight;
    mAnkleStance        = ljAnkleRight;
    mStanceFootContact  = rightFootContact;
    mHipSwing           = ljHipLeft;
    mKneeSwing          = ljKneeLeft;
    mAnkleSwing         = ljAnkleLeft;
    mSwingFootContact   = leftFootContact;
    // Calculate clearance for left foot
    mFootClearance      = std::min(leftToeZ, leftHeelZ);
    //mLogInfoLn("Foot clearance is " << mFootClearance);
  }

  // Adjust swing time, used to determine early swing against late swing
  mSwingTime += (uint64_t)(1.0E6/mDesiredFrequency);

  // Determine (virtual) footstep length. This doesn't mean that an actual footstep took place.
  // A footstep has positive length if the swing foot is in front of the stance foot
  // Here, we don't yet take into account that the stance leg might have changed.
  if (mLastStancelegWasLeft > 0)
    mFootstepLength = rightAnklePos - leftAnklePos;
  else
    mFootstepLength = leftAnklePos - rightAnklePos;

  // Determine whether footstep took place
  mMadeFootstep  = false;
  // A footstep is defined as a change of stance leg.
  // Therefore, footsteps can have positive *and* negative length
  if ((mLastStancelegWasLeft != leftIsStance) && (mLastStancelegWasLeft >= 0))
  {
    mMadeFootstep = true;
    // Adjust number of footsteps, but count negative footstep lengths as -1
    if (mFootstepLength > 0)
    {
      mNumFootsteps++;
      mSwingTime = 0;
    }
    else
      mNumFootsteps--;

    // Adjust walked length
    mWalkedDistance += mFootstepLength;

    mLogInfoLn("Number of footsteps increased to " << mNumFootsteps << "! Walked " << mWalkedDistance << " meters!");
  }

  // Adjust used energy
  mTrialEnergy += getEnergyUsage();

  /*
  // Requirements:
  // 1) One foot touches the floor in front of the other one, while it didn't the time step before.
  //    This must always be the (new) stance foot.
  if (mStanceFootContact)
  {
    // 2) This particular foot cannot make two footsteps in a row
    if (((mLastRewardedFoot != lpFootRight) && (mHipStance == ljHipRight)) ||
      ((mLastRewardedFoot != lpFootLeft) && (mHipStance == ljHipLeft)))
    {
      // We made a footstep!
      mMadeFootstep = true;
      mNumFootsteps++;

      // Determine length
      mFootstepLength        = stanceAnklePos - swingAnklePos;

      // Adjust walked length
      mWalkedDistance += mFootstepLength;
      mLogInfoLn("Number of footsteps increased to " << mNumFootsteps << "! Walked " << mWalkedDistance << " meters!");
    }
  }
  */

  // Record number of falls
  if (isDoomedToFall(getCurrentSTGState(), false))
    mNumFalls++;

  // Store this stanceleg in last stanceleg
  mLastStancelegWasLeft = (leftIsStance?1:0);
}

/*
double CLeoBhWalkSym::learn(char agentInTerminalState)
{
  double result = CSTGAgentQLeo::learn(agentInTerminalState);
  return result;
}*/

/*
void CLeoBhWalkSym::updateAction(ISTGActuation* actuationInterface)
{
  //printf("Before actuation: A[0]=%.3f, A[1]=%.3f, A[2]=%.3f\n", getCurrentSTGState()->mActuationVoltages[mHipStance], getCurrentSTGState()->mActuationVoltages[mHipSwing], getCurrentSTGState()->mActuationVoltages[mKneeSwing]);

  // Calculated actions
  autoActuateAnkles_FixedPos(actuationInterface);    // Fixed ankles
  autoActuateArm(actuationInterface);          // Fixed arm
  autoActuateKnees(actuationInterface);
  //autoActuateKnees_StanceLegFunc(actuationInterface);  // Fixed stance knee, but swing knee function
  if (mIsObserving)
    autoActuateHips2(NULL);                // Hips. Runs only during observing mode
  // Disturb the standard controller sometimes during observing mode -- only if learning is enabled (not for test runs)
  if (mIsObserving && mLearningEnabled)
    if (gRanrotB.Random() < mPreProgExploreRate)
      mAgentAction.randomizeUniform(&gRanrotB);

  // Put the agent's actions into effect
  actuationInterface->setJointVoltage(mHipStance,  mAgentAction[0]);
  actuationInterface->setJointVoltage(mHipSwing,  mAgentAction[1]);
  if (mLearnSwingKnee)
    actuationInterface->setJointVoltage(mKneeSwing,  mAgentAction[2]);

  if (true)
  {
    std::stringstream agentActionSs;
    agentActionSs << "Agent action: ";
    for (int i=0; i<mAgentAction.getNumActionVars(); i++)
      agentActionSs << i << ": " << mAgentAction.getActionVar(i) << ",\t";
    mLogDebugLn(agentActionSs.str());
  }

  //printf("Actual actions  : A[0]=%.3f, A[1]=%.3f, A[2]=%.3f\n", mAgentAction[0], mAgentAction[1], mAgentAction[2]);
  //printf("After actuation : A[0]=%.3f, A[1]=%.3f, A[2]=%.3f\n\n", getCurrentSTGState()->mActuationVoltages[mHipStance], getCurrentSTGState()->mActuationVoltages[mHipSwing], getCurrentSTGState()->mActuationVoltages[mKneeSwing]);

  // Calculate effective action if desired
  if (mUseEffectiveAction)
  {
    // Set effective action - ONLY valid for delay ratios of <= 1.0
    double delayRatio = actuationInterface->getComputationalDelayRatio();
    for (int i=0; i<LEOBHWALKSYM_MAX_NUM_ACTIONS; i++)
      mAgentEffectiveAction[i] = delayRatio*mPreviousAction[i] + (1.0-delayRatio)*mAgentAction[i];
    setExecutedActionType(atEffective);
    mLogInfoLn("Action (delay " << delayRatio << "):");
    mLogInfoLn("prev:" << mPreviousAction.toStr());
    mLogInfoLn("eff :" << mAgentEffectiveAction.toStr());
    mLogInfoLn("curr:" << mAgentAction.toStr());
  }

  // Use error leds to indicate stance leg
  actuationInterface->setLed(0, !mLastStancelegWasLeft);
  actuationInterface->setLed(1, false);
  actuationInterface->setLed(2, mLastStancelegWasLeft);

  //mLogInfoLn("Action: " << mAgentAction.toStr());
}
*/

/*
std::string CLeoBhWalkSym::getProgressReport(uint64_t absoluteTimeMicroseconds)
{
  std::stringstream progressString;
  // Life time
  //progressString << (double)(absoluteTimeMicroseconds - getResetMemoryTime())/(double)1.0E6 << "\t";
  // Use agent life duration, otherwise the time for standing up is included in the timeline
  progressString << (double)getAgentLifeDuration()/(double)1.0E6 << "\t";

  // Trial time
  double trialTime = (double)(absoluteTimeMicroseconds - getResetTime())/(double)1.0E6;
  progressString << trialTime << "\t";

  // Number of footsteps
  progressString << mNumFootsteps << "\t";

  // Number of cumulative falls since the birth of the agent
  progressString << mNumFalls << "\t";

  // Walked distance (estimate)
  progressString << mWalkedDistance << "\t";

  // Speed
  progressString << mWalkedDistance/trialTime << "\t";

  // Energy usage
  progressString << mTrialEnergy << "\t";

  // Energy per traveled meter
  if (mWalkedDistance != 0.0)
    progressString << mTrialEnergy/mWalkedDistance << "\t";
  else
    progressString << 0.0 << "\t";

  // Total reward - single agent
  progressString << getTotalReward() << "\t";

  // Total reward - multi-agent
  for (int iAgent=0; iAgent<getNumSubAgents(); iAgent++)
    progressString << getSubAgent(iAgent)->getTotalReward() << "\t";

  // Total memory usage
  _IndexPrecision totalMem = 0;
  if (getQSpace() != NULL)
    totalMem += getQSpace()->getMemoryUsage();
  for (int iSA=0; iSA<getNumSubAgents(); iSA++)
    totalMem += getSubAgent(iSA)->getQSpace()->getMemoryUsage();
  progressString << totalMem << "\t";

  // Memory usage of the main Q-space. Omit this value if there are no sub-agents, because then totalMem == QSpaceMem.
  if (getNumSubAgents() > 0)
  {
    if (getQSpace() != NULL)
      progressString << getQSpace()->getMemoryUsage() << "\t";
    else
      progressString << 0 << "\t";
  }

  // Memory usage per agent
  for (int iSA=0; iSA<getNumSubAgents(); iSA++)
    progressString << getSubAgent(iSA)->getQSpace()->getMemoryUsage() << "\t";

  // EOL
  progressString << "\n";

  return progressString.str();
}
*/

/*
void CLeoBhWalkSym::reset(uint64_t absoluteTimeMicroseconds)
{
  // Print memory usage and progress
  if (getQSpace() != NULL)
    mLogInfoLn("Memory usage: " << 100.0*(double)getQSpace()->getMemoryUsage()/(double)getQSpace()->getMemorySize() << "%");
  for (int iSA=0; iSA<getNumSubAgents(); iSA++)
    mLogInfoLn("Memory usage sub-agent " << iSA << ": " << 100.0*(double)getSubAgent(iSA)->getQSpace()->getMemoryUsage()/(double)getSubAgent(iSA)->getQSpace()->getMemorySize() << "%");
  if (mLearnRateDecayRate < 1.0)
    mLogInfoLn("Current learning rate: " << mLearnRate);


  mLogInfoLn("Learning: " << (mLearningEnabled?"enabled":"disabled") << ", Observing: " << (mIsObserving?"enabled":"disabled"));
  mLogInfo("Progress: " << getProgressReport(absoluteTimeMicroseconds) << flush);

  // Reset
  getActuationInterface()->setActuationMode(amVoltage);
  CSTGAgentQLeo::reset(absoluteTimeMicroseconds);
  mLastRewardedFoot     = lpFootLeft;
  mLastStancelegWasLeft = -1;
  mFootstepLength       = 0.0;
  mLastFootstepLength   = 0.0;
  // Are we observing?
  if (getAgentLifeDuration() < mObservingTime)
    mIsObserving = true;
  else
    mIsObserving = false;

  // Set mPreviousAction to 0.0 - this is probably close to the real robot's situation (although it doesn't matter much)
  for (int iAction=0; iAction<LEOBHWALKSYM_MAX_NUM_ACTIONS; iAction++)
    mPreviousAction[iAction] = 0.0;

  // Reset performance variables
  mNumFootsteps  = 0;
  mWalkedDistance  = 0.0;
  mTrialEnergy  = 0.0;

  // Init swing time so that robot starts to walk
  mSwingTime    = 0;
}
*/
/*
void CLeoBhWalkSym::resetMemory(uint64_t absoluteTimeMicroseconds)
{
  // Print memory usage as a warning to always come through
  if (getQSpace() != NULL)
    mLogWarningLn("[INFO] Memory usage: " << 100.0*(double)getQSpace()->getMemoryUsage()/(double)getQSpace()->getMemorySize() << "%");
  for (int iSA=0; iSA<getNumSubAgents(); iSA++)
    mLogWarningLn("[INFO] Memory usage sub-agent " << iSA << ": " << 100.0*(double)getSubAgent(iSA)->getQSpace()->getMemoryUsage()/(double)getSubAgent(iSA)->getQSpace()->getMemorySize() << "%");

  // Print final learn rate as well
  mLogWarningLn("[INFO] Final learn rate: " << mLearnRate);

  CSTGAgentQLeo::resetMemory(absoluteTimeMicroseconds);
  mNumFalls = 0;
}
*/

double CLeoBhWalkSym::getJointMotorWork(int jointIndex)
{
  if (getPreviousSTGState()->isValid())  // We don't have a previous state at the beginning of a trial
  {
    // Electrical work: P = U*I
    const double k = 0.00992;
    const double R = 8.6;
    const double G = 193.0;
    // We take the joint velocity as the average of the previous and the current velocity measurement
    double omega = 0.5*(getCurrentSTGState()->mJointSpeeds[jointIndex] + getPreviousSTGState()->mJointSpeeds[jointIndex]);
    // We take the action that was executed the previous step. This is reported in the *current* state
    double U = getCurrentSTGState()->mActuationVoltages[jointIndex];
    double I = (U - k*G*omega)/R;
    // Negative electrical work is not beneficial (no positive reward), but does not harm either.
    return std::max(0.0, U*I)/mDesiredFrequency;  // Divide power by frequency to get energy (work)
  }
  else
    return 0.0;
}

double CLeoBhWalkSym::getEnergyUsage()
{
  double leftHipWork   = getJointMotorWork(ljHipLeft);  //fabs(std::pow(mCurrentSTGState.mActuationTorques[ljHipLeft] - (0.00992*193)*mCurrentSTGState.mJointSpeeds[ljHipLeft], 2)/8.6);
  double rightHipWork  = getJointMotorWork(ljHipRight); //fabs(std::pow(mCurrentSTGState.mActuationTorques[ljHipRight] - (0.00992*193)*mCurrentSTGState.mJointSpeeds[ljHipRight], 2)/8.6);
  double leftKneeWork  = getJointMotorWork(ljKneeLeft); //fabs(std::pow(mCurrentSTGState.mActuationTorques[ljKneeLeft] - (0.00992*193)*mCurrentSTGState.mJointSpeeds[ljKneeLeft], 2)/8.6);
  double rightKneeWork = getJointMotorWork(ljKneeRight);//fabs(std::pow(mCurrentSTGState.mActuationTorques[ljKneeRight] - (0.00992*193)*mCurrentSTGState.mJointSpeeds[ljKneeRight], 2)/8.6);
  return leftHipWork + rightHipWork + leftKneeWork + rightKneeWork;
}

double CLeoBhWalkSym::getFootstepReward()
{
  double reward = 0;

  // Footstep reward
  if (mMadeFootstep)
  {
    // Only reward footsteps of certain maximum length. Don't use '> -maxFootstepLength', since the robot will make a large step backwards (no penalty) and a small step forward (reward) and walk backwards!
    reward += mRwFootstepDist*clip(mFootstepLength, -mRwFootstepMaxLength, mRwFootstepMaxLength);
    // Reward for specific footstep length:
    //double idealFootstepLength  = 0.18;
    //double footstepMargin    = 0.05;
    //result = mRwFootstepDist*idealFootstepLength*(1.0 - clip(fabs(mFootstepLength - idealFootstepLength)/footstepMargin, 0.0, 1.0));

    // Extra reward per event for footsteps backward
    if (mFootstepLength < 0)
      reward += mRwFootstepBackward;

    mLogNoticeLn("[REWARD] Robot made a footstep of " << mFootstepLength*100.0 << "cm! Reward = " << reward);
    //mAgentQLogger << " Stance hip: " << mHipStance << ", Stance contact: " << mStanceFootContact << ", swing contact: " << mSwingFootContact;
    if (mHipStance == ljHipRight)
      mLastRewardedFoot = lpFootRight;
    else
      mLastRewardedFoot = lpFootLeft;
  }

  // Continuous reward for changing distance between swing and stance foot
  if (mRwFootstepDistCont != 0)
  {
    double footDistChangeReward = mRwFootstepDistCont*(clip(mFootstepLength, -mRwFootstepMaxLength, mRwFootstepMaxLength) - clip(mLastFootstepLength, -mRwFootstepMaxLength, mRwFootstepMaxLength));
    reward += footDistChangeReward;
    //mLogInfoLn("Foot distance change reward: " << footDistChangeReward);
  }

  return reward;
}

double CLeoBhWalkSym::calculateReward()
{
  double reward = 0;
  // Time penalty
  reward += mRwTime;

  // Energy penalty
  if (mRwEnergy != 0.0)
  {
    double energyUsage = getEnergyUsage();
    reward += mRwEnergy*energyUsage;
    mLogDebugLn("[REWARD] Energy penalty: " << mRwEnergy*energyUsage);
  }

  // Footstep reward (calculation is a little bit more complicated -> separate function)
  reward += getFootstepReward();

  // Foot clearance reward
  if (mFootClearance < mRwFootClearanceThreshold)
  {
    double clearanceReward = mRwFootClearance*(mRwFootClearanceThreshold - mFootClearance);
    //mLogNoticeLn("[REWARD] Robot has low foot clearance of " << mFootClearance*100.0 << "cm! Reward = " << clearanceReward);
    reward += clearanceReward;
  }

  // Negative reward for 'falling' (doomed to fall)
  if (isDoomedToFall(getCurrentSTGState(), false))
  {
    reward += mRwDoomedToFall;
    mLogDebugLn("[REWARD] Doomed to fall! Reward: " << mRwDoomedToFall << " (total reward: " << getTotalReward() << ")" << endl);
  }

  // Reward for swinging the swing leg towards mid-stance
  if (getCurrentSTGState()->mJointAngles[mHipSwing] < getCurrentSTGState()->mJointAngles[mHipStance] + 0.3)
  {
    double swingReward = mRwHipAngleChange * (getCurrentSTGState()->mJointSpeeds[mHipSwing] - getCurrentSTGState()->mJointSpeeds[mHipStance]);
    mLogDebugLn("Swing reward: " << swingReward);
    reward += swingReward;
  }

  // Reward for keeping torso upright
  double torsoReward = mRwTorsoUpright * 1.0/(1.0 + (getCurrentSTGState()->mJointAngles[ljTorso] - mRwTorsoUprightAngle)*(getCurrentSTGState()->mJointAngles[ljTorso] - mRwTorsoUprightAngle)/(mRwTorsoUprightAngleMargin*mRwTorsoUprightAngleMargin));
  //mLogInfoLn("Torso upright reward: " << torsoReward);
  reward += torsoReward;

  // Penalty for both feet touching the floor
  if (mSwingFootContact && mStanceFootContact)
    reward += mRwDoubleStance;

  /*
  // Positive reward for forward hip velocity
  double stanceLegAngle    = mCurrentSTGState.mJointAngles[ljTorso] + mCurrentSTGState.mJointAngles[mHipStance];
  double stanceLegVel      = mCurrentSTGState.mJointSpeeds[ljTorso] + mCurrentSTGState.mJointSpeeds[mHipStance];
  double hipHorizontalMovementReward  = -2.0*stanceLegVel*cos(stanceLegAngle);
  double hipVerticalMovementReward  = 0;//-2.0*fabs(2.0*stanceLegVel*sin(stanceLegAngle));
  mAgentQLogger << "[REWARD] Hip moved: horizontal reward = " << hipHorizontalMovementReward << ", vertical reward = " << hipVerticalMovementReward << endl;
  reward += hipHorizontalMovementReward + hipVerticalMovementReward;
  */

//  if (mCurrentSTGState.mJointAngles[ljTorso] < -0.5)
//    reward += -6;
//  if (mCurrentSTGState.mJointAngles[ljTorso] > 0.1)
//    reward += -6;

//  if (mCurrentSTGState.mJointSpeeds[ljTorso] + mCurrentSTGState.mJointSpeeds[mHipStance] > 0)
//  {
//    //mAgentQLogger << "Hip stance speed: " << mCurrentSTGState.mJointSpeeds[ljTorso] + mCurrentSTGState.mJointSpeeds[mHipStance] << endl;
//    reward += -1;
//  }
//
//  if (mCurrentSTGState.mJointSpeeds[ljTorso] + mCurrentSTGState.mJointSpeeds[mHipSwing] < 0)
//  {
//    reward += -1;
//  }

  //mLogInfoLn("Step reward: " << reward);
  return reward;
}

/*
char CLeoBhWalkSym::isTerminalState(uint64_t trialTimeMs)
{
  if (isDoomedToFall(getCurrentSTGState(), true))
  {
    //mLogNoticeLn("[TERMINATION] Robot doomed to fall!");
    return CONST_STATE_TERMINAL | CONST_STATE_ABSORBING;
  }
  else
  if (trialTimeMs > mTrialTimeout)
  {
    mLogNoticeLn("[TERMINATION] Timeout of " << mTrialTimeout << "ms reached.");
    return CONST_STATE_TERMINAL;
  }
  else
    return CONST_STATE_NORMAL;
}
*/
bool CLeoBhWalkSym::isDoomedToFall(CLeoState* state, bool report)
{
  // Torso angle out of 'range'
  if ((state->mJointAngles[ljTorso] < -1.0) || (state->mJointAngles[ljTorso] > 1.0))
  {
    if (report)
      mLogNoticeLn("[TERMINATION] Torso angle too large");
    return true;
  }

  // No balancing
  /*
  double balanceVelocity = 0.02;
  if ((fabs(getCurrentSTGState()->mJointSpeeds[ljTorso]) < balanceVelocity)
    && (fabs(getCurrentSTGState()->mJointSpeeds[ljHipLeft]) < balanceVelocity)
    && (fabs(getCurrentSTGState()->mJointSpeeds[ljHipRight]) < balanceVelocity)
    )
  {
    if (report)
      mLogNoticeLn("[TERMINATION] Robot should not balance");
    return true;
  }
   */
  // Stance leg angle out of 'range'
  if (fabs(state->mJointAngles[ljTorso] + state->mJointAngles[mHipStance]) > 0.36*M_PI)
  {
    if (report)
      mLogNoticeLn("[TERMINATION] Stance leg angle too large");
    return true;
  }

  return false;
}

/*
void CLeoBhWalkSym::autoActuateHips(ISTGActuation* actuationInterface)
{
  // The "torque" here is not actually torque, but a leftover from the "endless turn mode" control from dynamixels, which is actually voltage control
  const double torqueToVoltage  = 14.0/3.3;
  // Hip: control the inter hip angle to a fixed angle
  double interHipAngleTorque  = 6.0*(0.62 - (getCurrentSTGState()->mJointAngles[mHipSwing] - getCurrentSTGState()->mJointAngles[mHipStance]));

  double hipStanceTorque    = (-0.40)*interHipAngleTorque;
  double hipSwingTorque    = interHipAngleTorque;

  if (mStanceFootContact)
  {
    // Torque to keep the upper body up right
     double stanceTorque = -14.0*(-0.17 - getCurrentSTGState()->mJointAngles[ljTorso]);
    hipStanceTorque      += stanceTorque;
  }
  if (mIsObserving)
  {
    mAgentAction[0] = torqueToVoltage*hipStanceTorque;
    mAgentAction[1] = torqueToVoltage*hipSwingTorque;
  }
  else
  {
    actuationInterface->setJointVoltage(mHipStance,  torqueToVoltage*hipStanceTorque);
    actuationInterface->setJointVoltage(mHipSwing,  torqueToVoltage*hipSwingTorque);
  }
}
*/
/*
void CLeoBhWalkSym::autoActuateHips2(ISTGActuation* actuationInterface)
{
  // The "torque" here is not actually torque, but a leftover from the "endless turn mode" control from dynamixels, which is actually voltage control
  const double torqueToVoltage  = 14.0/3.3;
  // Hip: control the inter hip angle to a fixed angle
  double interHipAngleTorque  = 4.0*(mPreProgHipAngle - (getCurrentSTGState()->mJointAngles[mHipSwing] - getCurrentSTGState()->mJointAngles[mHipStance]));

  double hipStanceTorque    = (-0.20)*interHipAngleTorque;
  double hipSwingTorque    = interHipAngleTorque;

  if (mStanceFootContact)
  {
    // Torque to keep the upper body up right
     double stanceTorque = -14.0*(mPreProgTorsoAngle - getCurrentSTGState()->mJointAngles[ljTorso]);
    hipStanceTorque      += stanceTorque;
  }
  if (mIsObserving)
  {
    double maxVoltage = getActuationInterface()->getJointMaxVoltage(ljHipLeft);
    mAgentAction[0] = clip(torqueToVoltage*hipStanceTorque, -maxVoltage, maxVoltage);
    mAgentAction[1] = clip(torqueToVoltage*hipSwingTorque, -maxVoltage, maxVoltage);
  }
  else
  {
    actuationInterface->setJointVoltage(mHipStance,  torqueToVoltage*hipStanceTorque);
    actuationInterface->setJointVoltage(mHipSwing,  torqueToVoltage*hipSwingTorque);
  }
}
*/
void CLeoBhWalkSym::autoActuateKnees(ISTGActuation* actuationInterface)
{
  // The stance knee contains a weak controller to remain stretched
  const double torqueToVoltage= 14.0/3.3;
  double kneeStanceTorque    = 5.0*(mPreProgStanceKneeAngle - getCurrentSTGState()->mJointAngles[mKneeStance]);
  double kneeSwingVoltage    = 0;
  if (mSwingTime < mPreProgEarlySwingTime)
  {
    // Early swing
    kneeSwingVoltage    = -14.0;  // Most probably clipped due to thermal guarantees
  }
  else
  {
    // Late swing
    kneeSwingVoltage    = 65.0*(mPreProgStanceKneeAngle - getCurrentSTGState()->mJointAngles[mKneeSwing]);
  }

  // Set joint voltages
  // Always set stance knee voltage
  getActuationInterface()->setJointVoltage(mKneeStance, torqueToVoltage*kneeStanceTorque);

  // When observing, set action to mAgentAction
  if (mIsObserving)
  {
    double maxVoltage = getActuationInterface()->getJointMaxVoltage(mKneeSwing);
    mAgentAction[2] = clip(kneeSwingVoltage, -maxVoltage, maxVoltage);
  }
  // When not observing and not learning the knee, set directly to actuation interface
  else if (!mLearnSwingKnee)
  {
    getActuationInterface()->setJointVoltage(mKneeSwing, kneeSwingVoltage);
  }
  // when not observing and actually learning the knee, do nothing
}

/*
void CLeoBhWalkSym::autoActuateKnees2(ISTGActuation* actuationInterface)
{
  // The "torque" here is not actually torque, but a leftover from the "endless turn mode" control from dynamixels, which is actually voltage control
  const double torqueToVoltage  = 14.0/3.3;
  // height of the swingleg ankle - relative to the leg length
  double relSwingKneeHeight    = cos(getCurrentSTGState()->mJointAngles[ljTorso] + getCurrentSTGState()->mJointAngles[mHipStance])
                  - 0.5*cos(getCurrentSTGState()->mJointAngles[ljTorso] + getCurrentSTGState()->mJointAngles[mHipSwing]);

  // The stance knee contains a weak controller to remain stretched
  double kneeStanceTorque    = 4.0*(0.0 - getCurrentSTGState()->mJointAngles[mKneeStance]);
  double kneeSwingTorque    = 0;
  // Calculate mid-stance
  double stanceLegAngle    = getCurrentSTGState()->mJointAngles[ljTorso] + getCurrentSTGState()->mJointAngles[mHipStance];

  double relClearance = 0.45;
  double upperSwingLegAngle    = getCurrentSTGState()->mJointAngles[ljTorso] + getCurrentSTGState()->mJointAngles[mHipSwing];
  double swingKneeClearanceAngle;
  if (stanceLegAngle > -0.08)  // Accomplish swing foot clearance
  {
    swingKneeClearanceAngle = upperSwingLegAngle - acos(clip(2*(relSwingKneeHeight - relClearance), -1.0, 1.0));
  }
  else  // Stretch swing leg
  {
    swingKneeClearanceAngle = 0;
  }
  kneeSwingTorque = 14*(swingKneeClearanceAngle - getCurrentSTGState()->mJointAngles[mKneeSwing]);

  // Set joint voltages
  getActuationInterface()->setJointVoltage(mKneeStance,  torqueToVoltage*kneeStanceTorque);
  getActuationInterface()->setJointVoltage(mKneeSwing,  torqueToVoltage*kneeSwingTorque);
}
*/
/*
void CLeoBhWalkSym::autoActuateKnees_StanceLegFunc(ISTGActuation* actuationInterface)
{
  // The "torque" here is not actually torque, but a leftover from the "endless turn mode" control from dynamixels, which is actually voltage control
  const double torqueToVoltage  = 14.0/3.3;

  double upperStanceLegAngle    = getCurrentSTGState()->mJointAngles[ljTorso] + getCurrentSTGState()->mJointAngles[mHipStance];
  double theta1          = 0.25;
  double theta2          = 0.5;
  double thetaShift        = 0.1;
  double maxKneeFlection      = -0.50*M_PI;
  double desiredSwingKneeAngle;
  if (upperStanceLegAngle < thetaShift)
    desiredSwingKneeAngle = maxKneeFlection*cos(0.5*M_PI*clip((upperStanceLegAngle - thetaShift)/theta1, -1.0, 1.0));
  else
    desiredSwingKneeAngle = maxKneeFlection*cos(0.5*M_PI*clip((upperStanceLegAngle - thetaShift)/theta2, -1.0, 1.0));


  // The stance knee contains a weak controller to remain stretched
  double kneeStanceTorque    = 5.0*(0.0 - getCurrentSTGState()->mJointAngles[mKneeStance]);
  double kneeSwingTorque    = 10.0*(desiredSwingKneeAngle - getCurrentSTGState()->mJointAngles[mKneeSwing]);
  //mAgentQLogger << "[INFO] KneeSwingTorque: " << kneeSwingTorque << endl;
  // Set joint voltages
  // Always set stance knee voltage
  getActuationInterface()->setJointVoltage(mKneeStance,  torqueToVoltage*kneeStanceTorque);

  // When observing, set action to mAgentAction
  if (mIsObserving)
  {
    double maxVoltage = getActuationInterface()->getJointMaxVoltage(ljKneeLeft);
    mAgentAction[2] = clip(torqueToVoltage*kneeSwingTorque, -maxVoltage, maxVoltage);
  }
  // When not observing and not learning the knee, set directly to actuation interface
  else if (!mLearnSwingKnee)
  {
    getActuationInterface()->setJointVoltage(mKneeSwing,  torqueToVoltage*kneeSwingTorque);
  }
  // when not observing and learning the knee, do nothing
}
*/
/*
void CLeoBhWalkSym::autoActuateAnkles(ISTGActuation* actuationInterface)
{
  // The "torque" here is not actually torque, but a leftover from the "endless turn mode" control from dynamixels, which is actually voltage control
  const double torqueToVoltage  = 14.0/3.3;

  // Determine appropriate torques
  double swingFootFloorAngle  = getCurrentSTGState()->mJointAngles[ljTorso] + getCurrentSTGState()->mJointAngles[mHipSwing] + getCurrentSTGState()->mJointAngles[mKneeSwing] + getCurrentSTGState()->mJointAngles[mAnkleSwing];
  double stanceFootFloorAngle  = getCurrentSTGState()->mJointAngles[ljTorso] + getCurrentSTGState()->mJointAngles[mHipStance] + getCurrentSTGState()->mJointAngles[mKneeStance] + getCurrentSTGState()->mJointAngles[mAnkleStance];
  //double interHipAngle    = getCurrentSTGState()->mJointAngles[mHipSwing] - getCurrentSTGState()->mJointAngles[mHipStance];

  // The ankles use a P-controller to bring the angle between foot and floor to a fixed angle
  //double ankleSwingTorque    = 14.0*(clip(interHipAngle/0.35, -1.0, 1.0)*0.10 - swingFootFloorAngle);
  double ankleSwingTorque    = 8.0*(mPreProgAnkleAngle - swingFootFloorAngle);
  //double ankleStanceTorque  = 5.0*(0.065 - stanceFootFloorAngle);
  double ankleStanceTorque  = 8.0*(mPreProgAnkleAngle - stanceFootFloorAngle);

  // Set joint voltages
  getActuationInterface()->setJointVoltage(mAnkleStance,  torqueToVoltage*ankleStanceTorque);
  getActuationInterface()->setJointVoltage(mAnkleSwing,  torqueToVoltage*ankleSwingTorque);
}
*/
void CLeoBhWalkSym::autoActuateAnkles_FixedPos(ISTGActuation* actuationInterface)
{
  // The "torque" here is not actually torque, but a leftover from the "endless turn mode" control from dynamixels, which is actually voltage control
  const double torqueToVoltage  = 14.0/3.3;

  double K          = 10.0*torqueToVoltage;
  double D          = 0;//0.00992*193.0*1.1;
  double leftAnkleTorque    = K*(mPreProgAnkleAngle - getCurrentSTGState()->mJointAngles[ljAnkleLeft]) + D*getCurrentSTGState()->mJointSpeeds[ljAnkleLeft];
  double rightAnkleTorque    = K*(mPreProgAnkleAngle - getCurrentSTGState()->mJointAngles[ljAnkleRight]) + D*getCurrentSTGState()->mJointSpeeds[ljAnkleRight];
  // Set joint voltages
  getActuationInterface()->setJointVoltage(ljAnkleLeft,  leftAnkleTorque);
  getActuationInterface()->setJointVoltage(ljAnkleRight,  rightAnkleTorque);
}


void CLeoBhWalkSym::autoActuateArm(ISTGActuation* actuationInterface)
{
  // The "torque" here is not actually torque, but a leftover from the "endless turn mode" control from dynamixels, which is actually voltage control
  const double torqueToVoltage  = 14.0/3.3;

  double armTorque = 5.0*(mPreProgShoulderAngle - getCurrentSTGState()->mJointAngles[ljShoulder]);
  getActuationInterface()->setJointVoltage(ljShoulder, torqueToVoltage*armTorque);
}

// ********************************************************************* //
// ************************* gCreateLeoBhWalkSym *********************** //
// ********************************************************************* //
/*
CLeoBhWalkSym* gCreateLeoBhWalkSym(ISTGActuation *actuationInterface, const CConfigSection &policyConfigNode)
{
  return new CLeoBhWalkSym(actuationInterface);
}
*/
