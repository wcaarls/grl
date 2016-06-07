#include <XMLConfiguration.h>
#include <grl/environments/leo/leo_squat.h>

using namespace grl;

REGISTER_CONFIGURABLE(LeoSquatEnvironment)

//const double T[] = { 0.0,  0.0,  0.0,  0.0,  0.0};
//const double B[] = { 1.4,  1.4, -1.9, -1.9, -0.3}; // hips, knees, torso

#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))

const double T = 0.26;
const double B = 0.18;

void CLeoBhSquat::resetState()
{
  CLeoBhBase::resetState();
  prev_direction_ = direction_ = -1;
}

double CLeoBhSquat::calculateReward()
{
  double reward = 0;
  // Negative reward for 'falling' (doomed to fall)
  if (isDoomedToFall(getCurrentSTGState(), false))
  {
    reward += mRwDoomedToFall;
    mLogDebugLn("[REWARD] Doomed to fall! Reward: " << mRwDoomedToFall << " (total reward: " << getTotalReward() << ")" << endl);
  }
  else
  {
    double rw;
    if (direction_ == -1)
      rw = pow(cHipHeight_ - B, 2) - pow(pHipHeight_ - B, 2);
    else if (direction_ == 1)
      rw = pow(cHipHeight_ - T, 2) - pow(pHipHeight_ - T, 2);
    reward += -100*rw;

    std::cout << pHipHeight_ << " -> " << cHipHeight_ << " = " << reward << std::endl;

    if (direction_ != prev_direction_)
      reward += 10;

    // Reward for keeping torso upright
    //double torsoReward = mRwTorsoUpright * 1.0/(1.0 + (s->mJointAngles[ljTorso] - mRwTorsoUprightAngle)*(s->mJointAngles[ljTorso] - mRwTorsoUprightAngle)/(mRwTorsoUprightAngleMargin*mRwTorsoUprightAngleMargin));
    //reward += torsoReward;
  }

  return reward;
}

void CLeoBhSquat::getHipHeight(const double *x, double &hipHeight, double &hipPos) const
{
  // Determine foot position relative to the hip axis
  double upLegLength        = 0.116;  // length of the thigh
  double loLegLength        = 0.1045; // length of the shin
  double leftHipAbsAngle    = x[ljTorso] + x[ljHipLeft];
  double leftKneeAbsAngle   = leftHipAbsAngle + x[ljKneeLeft];
  double leftAnkleAbsAngle  = leftKneeAbsAngle + x[ljAnkleLeft];
  double rightHipAbsAngle   = x[ljTorso] + x[ljHipRight];
  double rightKneeAbsAngle  = rightHipAbsAngle + x[ljKneeRight];
  double rightAnkleAbsAngle = rightKneeAbsAngle + x[ljAnkleRight];
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

  hipHeight = std::max(std::max(leftHeelZ, leftToeZ), std::max(rightHeelZ, rightToeZ));
  hipPos = rightAnklePos;

  //TRACE("Hip height: " << hh);
  //std::cout << "Hip height: " << hipHeight << std::endl;
  //std::cout << "Ankle pos: " << hipPos << std::endl;
}

bool CLeoBhSquat::isSitting() const
{
  if (cHipHeight_ < B)
    return true;
  return false;
}

bool CLeoBhSquat::isStanding() const
{
  if (cHipHeight_ > T)
    return true;
  return false;
}

void CLeoBhSquat::parseLeoState(const CLeoState &leoState, Vector &obs)
{
  obs[osTorsoAngle]           = leoState.mJointAngles[ljTorso];
  obs[osTorsoAngleRate]       = leoState.mJointSpeeds[ljTorso];
  obs[osHipStanceAngle]       = leoState.mJointAngles[mHipStance];
  obs[osHipStanceAngleRate]   = leoState.mJointSpeeds[mHipStance];
  obs[osKneeStanceAngle]      = leoState.mJointAngles[mKneeStance];
  obs[osKneeStanceAngleRate]  = leoState.mJointSpeeds[mKneeStance];

  // calculate hip locations
  pHipHeight_ = cHipHeight_;
  pHipPos_ = cHipPos_;
  getHipHeight(getCurrentSTGState()->mJointAngles, cHipHeight_, cHipPos_);

  prev_direction_ = direction_;
  if (isStanding())
    std::cout << "Is standing" << std::endl;
  if (isSitting())
    std::cout << "Is sitting" << std::endl;

  if (direction_ == -1 && isSitting())
    obs[osDirection] = direction_ =  1;
  else if (direction_ == 1 && isStanding())
    obs[osDirection] = direction_ = -1;
}

bool CLeoBhSquat::isDoomedToFall(CLeoState* state, bool report)
{
  // Torso angle out of 'range'
  if ((state->mJointAngles[ljTorso] < -1.0) || (state->mJointAngles[ljTorso] > 1.0)  || fabs(cHipPos_) > 0.14) // state->mFootContacts == 0
  {
    if (report)
      mLogNoticeLn("[TERMINATION] Torso angle too large");
    return true;
  }
  return false;
}
/////////////////////////////////

LeoSquatEnvironment::LeoSquatEnvironment()
{
  bh_ = new CLeoBhSquat(&leoSim_);
  set_bh(bh_);
}

void LeoSquatEnvironment::request(ConfigurationRequest *config)
{
  LeoBaseEnvironment::request(config);
}

void LeoSquatEnvironment::configure(Configuration &config)
{
  LeoBaseEnvironment::configure(config);

  // Augmenting state with a direction indicator variable: sit down or stand up
  observation_dims_++;
  config.set("observation_dims", observation_dims_);
  Vector new_obs_min, new_obs_max;
  new_obs_min.resize(observation_dims_);
  new_obs_max.resize(observation_dims_);
  new_obs_min << config["observation_min"].v(), VectorConstructor(-1);
  new_obs_max << config["observation_max"].v(), VectorConstructor(+1);
  config.set("observation_min", new_obs_min);
  config.set("observation_max", new_obs_max);
}

LeoSquatEnvironment *LeoSquatEnvironment::clone() const
{
  return NULL;
}

void LeoSquatEnvironment::start(int test, Vector *obs)
{
  target_env_->start(test, &target_obs_);

  // Parse obs into CLeoState (Start with left leg being the stance leg)
  bh_->resetState();
  bh_->fillLeoState(target_obs_, Vector(), leoState_);
  bh_->setCurrentSTGState(&leoState_);
  bh_->setPreviousSTGState(&leoState_);

  // update derived state variables
  bh_->updateDerivedStateVars(&leoState_); // swing-stance switching happens here

  // construct new obs from CLeoState
  obs->resize(observation_dims_);
  bh_->parseLeoState(leoState_, *obs);

  bh_->setCurrentSTGState(NULL);

  LeoBaseEnvironment::start(test);
}

double LeoSquatEnvironment::step(const Vector &action, Vector *obs, double *reward, int *terminal)
{
  TRACE("RL action: " << action);

  bh_->setCurrentSTGState(&leoState_);

  double actionArm = bh_->grlAutoActuateArm();
  target_action_ << actionArm, action[0], action[0], action[1], action[1], action[2], action[2];

  bh_->setPreviousSTGState(&leoState_);
  double tau = target_env_->step(target_action_, &target_obs_, reward, terminal);

  // Filter joint speeds
  // Parse obs into CLeoState
  bh_->fillLeoState(target_obs_, target_action_, leoState_);
  bh_->setCurrentSTGState(&leoState_);

  // update derived state variables
  bh_->updateDerivedStateVars(&leoState_);

  // construct new obs from CLeoState
  bh_->parseLeoState(leoState_, *obs);

  // Determine reward
  *reward = bh_->calculateReward();

  // ... and termination
  if (*terminal == 1) // timeout
    *terminal = 1;
  else if (bh_->isDoomedToFall(&leoState_, false))
    *terminal = 2;
  else
    *terminal = 0;

  LeoBaseEnvironment::step(tau, *reward, *terminal);
  return tau;
}

