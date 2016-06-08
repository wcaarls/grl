/*
 * LeoBhWalk.cpp
 *
 *  Created on: Sep 1, 2009
 *      Author: Erik Schuitema
 */

#include "LeoBhWalkSym.h"
#include <cmath>
#include <bitset>

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
  mPrintState(false),
  mMadeFootstep(false)
{
  mDesiredFrequency      = 30.0;
  mDesiredMemorySize      = 1024*1024*4;
  mDesiredNumTilings      = 16;
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
}


bool CLeoBhWalkSym::readConfig(const CConfigSection &xmlRoot)
{
  CConfigSection configNode = xmlRoot.section("policy");

  bool configresult = true;
  configresult &= mLogAssert(configNode.get("frequency", &mDesiredFrequency));
  configresult &= mLogAssert(configNode.get("memorySize", &mDesiredMemorySize));
  configresult &= mLogAssert(configNode.get("numTilings", &mDesiredNumTilings));

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
  configresult &= mLogAssert(configNode.get("preprogrammedTorsoAngle", &mPreProgTorsoAngle));
  configresult &= mLogAssert(configNode.get("preprogrammedHipAngle", &mPreProgHipAngle));
  configresult &= mLogAssert(configNode.get("preprogrammedShoulderAngle", &mPreProgShoulderAngle));
  configresult &= mLogAssert(configNode.get("preprogrammedAnkleAngle", &mPreProgAnkleAngle));

  // ivan: The following values are not found in XML
  //mLogAssert(configNode.get("preprogrammedStanceKneeAngle", &mPreProgStanceKneeAngle));
  //mLogAssert(configNode.get("preprogrammedEarlySwingTime", &mPreProgEarlySwingTime));
  //mLogAssert(configNode.get("preprogrammedExploreRate", &mPreProgExploreRate));


  double timeSeconds=0;
  configresult &= mLogAssert(configNode.get("trialTimeoutSeconds", &timeSeconds));
  mTrialTimeout = (uint64_t)(timeSeconds*1E6);
  configresult &= mLogAssert(configNode.get("observingTimeSeconds", &timeSeconds));
  mObservingTime = (uint64_t)(timeSeconds*1E6);
  configresult &= mLogAssert(configNode.get("useEffectiveAction",&mUseEffectiveAction));
  configresult &= mLogAssert(configNode.get("generalizeActions", &mGeneralizeActions));

  configresult &= mLogAssert(configNode.get("scaleFactTorsoAngle",			&mScaleFactTorsoAngle));
  configresult &= mLogAssert(configNode.get("scaleFactTorsoAngleRate",		&mScaleFactTorsoAngleRate));
  configresult &= mLogAssert(configNode.get("scaleFactHipStanceAngle",		&mScaleFactHipStanceAngle));
  configresult &= mLogAssert(configNode.get("scaleFactHipStanceAngleRate",	&mScaleFactHipStanceAngleRate));
  configresult &= mLogAssert(configNode.get("scaleFactHipSwingAngle",			&mScaleFactHipSwingAngle));
  configresult &= mLogAssert(configNode.get("scaleFactHipSwingAngleRate",		&mScaleFactHipSwingAngleRate));
  configresult &= mLogAssert(configNode.get("scaleFactKneeStanceAngle",		&mScaleFactKneeStanceAngle));
  configresult &= mLogAssert(configNode.get("scaleFactKneeStanceAngleRate",	&mScaleFactKneeStanceAngleRate));
  configresult &= mLogAssert(configNode.get("scaleFactKneeSwingAngle", 		&mScaleFactKneeSwingAngle));
  configresult &= mLogAssert(configNode.get("scaleFactKneeSwingAngleRate",	&mScaleFactKneeSwingAngleRate));

  configNode.get("multiResScaleFact", &mMultiResScaleFact);

  configresult &= mLogAssert(configNode.get("numActionsPerJoint", &mNumActionsPerJoint));
  configresult &= mLogAssert(configNode.get("scaleFactVoltage",	&mScaleFactVoltage));

  /////////////
  configNode = xmlRoot.section("ode");
  configNode.get("steptime", &mTotalStepTime);

  return configresult;
}

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
  }
  mLogInfoLn("Foot clearance is " << mFootClearance);
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
  double leftHipWork   = getJointMotorWork(ljHipLeft);
  double rightHipWork  = getJointMotorWork(ljHipRight);
  double leftKneeWork  = getJointMotorWork(ljKneeLeft);
  double rightKneeWork = getJointMotorWork(ljKneeRight);
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
//  else if (!mLearnSwingKnee)
//  {
//    getActuationInterface()->setJointVoltage(mKneeSwing, kneeSwingVoltage);
//  }
  // when not observing and actually learning the knee, do nothing
}

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

std::string CLeoBhWalkSym::getProgressReport(double trialTime)
{
  const int pw = 15;

  std::stringstream progressString;

  progressString << std::fixed << std::setprecision(3) << std::right;

  // Trial time
  progressString << std::setw(pw) << trialTime;

  // Number of footsteps in this trial
  progressString << std::setw(pw) << mNumFootsteps;

  // Number of cumulative falls since the birth of the agent
  progressString << std::setw(pw) << mNumFalls;

  // Walked distance (estimate)
  progressString << std::setw(pw) << mWalkedDistance;

  // Speed
  progressString << std::setw(pw) << mWalkedDistance/trialTime;

  // Energy usage
  progressString << std::setw(pw) << mTrialEnergy;

  // Energy per traveled meter
  if (mWalkedDistance > 0.001)
    progressString << std::setw(pw) << mTrialEnergy/mWalkedDistance;
  else
    progressString << std::setw(pw) << 0.0;

  return progressString.str();
}
