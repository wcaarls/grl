#include <XMLConfiguration.h>
#include <grl/environments/leo/leo_squat.h>

using namespace grl;

REGISTER_CONFIGURABLE(LeoSquatEnvironment)

#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))

const double T = 0.36;
const double B = 0.28;

void CLeoBhSquat::resetState(double time0)
{
  CLeoBhBase::resetState();

  getHipHeight(getCurrentSTGState()->mJointAngles, hip_height_, hip_pos_);

  if (hip_height_ < 0.5*(T+B))
    prev_direction_ = direction_ = 1;
  else
    prev_direction_ = direction_ = -1;

  squat_counter_ = 0;
  time_of_dir_change_ = time0;
  up_time_.clear();
  down_time_.clear();
  max_hip_height_ = min_hip_height_ = hip_height_;
}

double CLeoBhSquat::calculateReward()
{
  double reward = 0;
  // Negative reward for 'falling' (doomed to fall)
  if (isDoomedToFall(getCurrentSTGState(), false))
  {
    reward += -100;
    mLogDebugLn("[REWARD] Doomed to fall! Reward: " << mRwDoomedToFall << " (total reward: " << getTotalReward() << ")" << endl);
  }
  else
  {
    double taskReward = 0, energyReward = 0, feetReward = 0, velocityReward = 0;
    //if ( (direction_ == -1 && isSitting()) || (direction_ == 1 && isStanding()) )
    //  reward = 6;
    //else
    //{

      // Energy
      //double ankleLeftWork  = getJointMotorWork(ljAnkleLeft);
      //double ankleRightWork = getJointMotorWork(ljAnkleRight);
      //energyReward = -0.05 * (getEnergyUsage() + ankleLeftWork + ankleRightWork);

      // Feet lifting
      //if (getCurrentSTGState()->mFootContacts != 15)
      //  feetReward = -10;


      // Task
      bool shaping = true;
      if (shaping)
      {
        if (direction_ == -1)
          taskReward = pow(hip_height_ - B, 2) - pow(prev_hip_height_ - B, 2);
         else if (direction_ == 1)
          taskReward = pow(hip_height_ - T, 2) - pow(prev_hip_height_ - T, 2);
        taskReward = -3000*taskReward;
      }
      else
      {
        if (direction_ == -1)
          taskReward = pow(hip_height_ - B, 2);
         else if (direction_ == 1)
          taskReward = pow(hip_height_ - T, 2);
        taskReward *= -1.0;
      }

      const double *v = getCurrentSTGState()->mJointSpeeds;
      velocityReward =  v[ljTorso]*v[ljTorso] +
                        v[ljShoulder]*v[ljShoulder] +
                        2*v[ljHipRight]*v[ljHipRight] +
                        2*v[ljKneeRight]*v[ljKneeRight] +
                        2*v[ljAnkleRight]*v[ljAnkleRight];
      velocityReward *= -0.02;


      const double *u = getCurrentSTGState()->mActuationVoltages;
      energyReward =  u[ljHipRight]*u[ljHipRight] +
                      u[ljKneeRight]*u[ljKneeRight] +
                      u[ljAnkleRight]*u[ljAnkleRight];
      energyReward *= -0.0001;

//      std::cout << taskReward << ", " << energyReward << " , " << velocityReward << std::endl;

      reward = energyReward + taskReward + feetReward + velocityReward;
    //}

    //std::cout << prev_hip_height_ << " -> " << hip_height_ << " = " << taskReward << ", " << energyReward << " , " << feetReward << " = " << reward << std::endl;

    // Reward for keeping torso upright
    //double torsoReward = mRwTorsoUpright * 1.0/(1.0 + (s->mJointAngles[ljTorso] - mRwTorsoUprightAngle)*(s->mJointAngles[ljTorso] - mRwTorsoUprightAngle)/(mRwTorsoUprightAngleMargin*mRwTorsoUprightAngleMargin));
    //reward += torsoReward;
  }

  return reward;
}

void CLeoBhSquat::getHipHeight(const double *x, double &hipHeight, double &hipPos) const
{
  // Determine foot position relative to the hip axis
  const double torsoHeight  = 0.24155;
  const double upLegLength  = 0.116;  // length of the thigh
  const double loLegLength  = 0.1045; // length of the shin


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

  hipHeight = std::max(std::max(leftHeelZ, leftToeZ), std::max(rightHeelZ, rightToeZ)) + (torsoHeight/2)*cos(x[ljTorso]);
  hipPos = rightAnklePos;

  //TRACE("Hip height: " << hh);
  //std::cout << "Hip height: " << hipHeight << std::endl;
  //std::cout << "Ankle pos: " << hipPos << std::endl;
}

void CLeoBhSquat::getCOM(const double *x, double &hipHeight, double &hipPos) const
{
  // Determine foot position relative to the hip axis
  const double torsoHeight  = 0.24155;
  const double upLegLength  = 0.116;
  const double loLegLength  = 0.1045;
  const double footwheelradius = 0.013;
  //const double shoulderLength = ;


  const double torsoMass = 0.91326;
  const double torsoCMZ  = 0.009945;
  const double boomCMY = 0.835;
  const double boomMass = 0.860;
  const double boomIZZ = 0.31863;
  const double boomLength = 1.70;
  const double boomVirtualMassX = (boomCMY*boomCMY*boomMass + boomIZZ)/(boomLength*boomLength);
  const double torsoHipDistZ = -torsoHeight/2;

  const double torsoComZ    = (torsoMass*torsoCMZ + boomVirtualMassX*torsoHipDistZ)/(torsoMass + boomVirtualMassX);
  const double upLegComZ    = -0.00481;
  const double loLegComZ    = -0.00867;
  const double footComZ     = 0.00461;
  const double shoulderComZ   = -0.14260;
  const double shoulderAnchorZ = 0.091275;

  const double torsoBoomMass  = torsoMass + boomVirtualMassX;
  const double upLegMass  = 0.17978;
  const double loLegMass  = 0.12691;
  const double footMass = 0.07319;
  const double shoulderMass = 0.095;

  double torsoZ = footwheelradius + loLegLength + upLegLength + torsoHeight/2 + torsoComZ;
  double upLegZ = footwheelradius + loLegLength + upLegLength/2 + upLegComZ;
  double loLegZ = footwheelradius + loLegLength/2 + loLegComZ;
  double footZ  = footwheelradius + footComZ;
  double shoulderZ = torsoZ + shoulderComZ + shoulderAnchorZ;

  double z = (torsoZ*torsoBoomMass + 2*upLegZ*upLegMass + 2*loLegZ*loLegMass + 2*footZ*footMass + shoulderZ*shoulderMass) /
      (torsoBoomMass + 2*upLegMass + 2*loLegMass + 2*footMass + shoulderMass);
  std::cout << z << std::endl;
}

bool CLeoBhSquat::isSitting() const
{
  if (hip_height_ < B)
    return true;
  return false;
}

bool CLeoBhSquat::isStanding() const
{
  if (hip_height_ > T)
    return true;
  return false;
}

void CLeoBhSquat::parseLeoState(const CLeoState &leoState, Vector &obs)
{
  int i, j;
  for (i = 0; i < observer_struct_.angles.size(); i++)
    obs[i] = leoState.mJointAngles[ observer_struct_.angles[i] ];
  for (j = 0; j < observer_struct_.angle_rates.size(); j++)
    obs[i+j] = leoState.mJointSpeeds[ observer_struct_.angle_rates[j] ];
  for (int k = 0; k < observer_struct_.augmented.size(); k++)
  {
    if (observer_struct_.augmented[k] == "direction")
      obs[i+j+k] = direction_;
    else if (observer_struct_.augmented[k] == "heeltoe")
      obs[i+j+k] = (leoState.mFootContacts == 0?0:1);
    else
    {
      ERROR("Unknown augmented field '" << observer_struct_.augmented[i] << "'");
      throw bad_param("leo_squat:observer_idx_.augmented[i]");
    }
  }

  // update hip locations
  prev_hip_height_ = hip_height_;
  prev_hip_pos_ = hip_pos_;
  getHipHeight(getCurrentSTGState()->mJointAngles, hip_height_, hip_pos_);
  min_hip_height_ = MIN(min_hip_height_, hip_height_);
  max_hip_height_ = MAX(max_hip_height_, hip_height_);

  //std::cout << "Hip height: " << hip_height_ << std::endl;
}

void CLeoBhSquat::updateDirection(double time)
{
  prev_direction_ = direction_;

  //if (isStanding())
  //  std::cout << "Is standing" << std::endl;
  //if (isSitting())
  //  std::cout << "Is sitting" << std::endl;

  if (direction_ == -1 && isSitting())
    direction_ =  1;
  else if (direction_ == 1 && isStanding())
    direction_ = -1;

  if (prev_direction_ != direction_)
  {
    squat_counter_++;
    if (prev_direction_ == 1)
      up_time_.push_back(time-time_of_dir_change_);
    else
      down_time_.push_back(time-time_of_dir_change_);
    time_of_dir_change_ = time;
  }
}

bool CLeoBhSquat::isDoomedToFall(CLeoState* state, bool report)
{
  double feet_angle = state->mJointAngles[ljHipRight] + state->mJointAngles[ljKneeRight] +
                      state->mJointAngles[ljAnkleRight] + state->mJointAngles[ljTorso];
  //std::cout << "Contact angle: " << feet_angle << std::endl;

  // Torso angle out of 'range'
  //if ((state->mJointAngles[ljTorso] < -1.4) || (state->mJointAngles[ljTorso] > 1.4) )//|| state->mFootContacts != 15) // state->mFootContacts == 0
  //if ((state->mJointAngles[ljTorso] < -1.4) || (state->mJointAngles[ljTorso] > 1.4) || (fabs(feet_angle) > 0.03) || (state->mFootContacts == 0))//|| (state->mFootContacts != 15))
  if ((state->mJointAngles[ljTorso] < -1.4) || (state->mJointAngles[ljTorso] > 1.4))
  //if ((state->mJointAngles[ljTorso] < -1.4) || (state->mJointAngles[ljTorso] > 1.4)) //|| (hip_height_ < 0.24))
  {
    if (report)
      TRACE("[TERMINATION] Torso angle is too large");
    return true;
  }
  return false;
}

std::string CLeoBhSquat::getProgressReport(double trialTime)
{
  const int pw = 15;
  std::stringstream progressString;
  progressString << std::fixed << std::setprecision(3) << std::right;

  // Squat counter
  progressString << std::setw(pw) << squat_counter_;

  Vector v;
  double m, s;
  // Robot rising time mean and variance
  if (up_time_.size())
  {
    toVector(up_time_, v);
    m = v.mean();
    s = dot(v-m, v-m) / v.size();
    progressString << std::setw(pw) << m << std::setw(pw) << s;
  }
  else
    progressString << std::setw(pw) << 0 << std::setw(pw) << 0;

  // Robot squatting time mean and variance
  if (down_time_.size())
  {
    toVector(down_time_, v);
    m = v.mean();
    s = dot(v-m, v-m) / v.size();
    progressString << std::setw(pw) << m << std::setw(pw) << s;
  }
  else
    progressString << std::setw(pw) << 0 << std::setw(pw) << 0;

  // Max and min hip heights
  progressString << std::setw(pw) << min_hip_height_ << std::setw(pw) << max_hip_height_;

  return progressString.str();
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
  const ObserverStruct &os = bh_->getObserverStruct();
  Vector obs_min = config["observation_min"].v();
  Vector obs_max = config["observation_max"].v();

  for (int i = 0; i < os.augmented.size(); i++)
  {
    if (os.augmented[i] == "direction")
    {
      obs_min[os.angles.size()+os.angle_rates.size() + i] = -1;
      obs_max[os.angles.size()+os.angle_rates.size() + i] = +1;
    }
    else if (os.augmented[i] == "heeltoe")
    {
      obs_min[os.angles.size()+os.angle_rates.size() + i] =  0;
      obs_max[os.angles.size()+os.angle_rates.size() + i] =  1;
    }
    else
    {
      ERROR("Unknown augmented field '" << os.augmented[i] << "'");
      throw bad_param("leo_squat:os.augmented[i]");
    }
  }

  config.set("observation_min", obs_min);
  config.set("observation_max", obs_max);

  TRACE("Observation min: " << obs_min);
  TRACE("Observation max: " << obs_max);
}

LeoSquatEnvironment *LeoSquatEnvironment::clone() const
{
  return NULL;
}

void LeoSquatEnvironment::start(int test, Vector *obs)
{
  LeoBaseEnvironment::start(test);

  target_env_->start(test_, &target_obs_);

  //double com, com1;
  //bh_->getCOM(NULL, com, com1);

  // Parse obs into CLeoState (Start with left leg being the stance leg)
  bh_->fillLeoState(target_obs_, Vector(), leoState_);
  bh_->setCurrentSTGState(&leoState_);
  bh_->setPreviousSTGState(&leoState_);
  bh_->resetState(test_?time_test_:time_learn_);

  // update derived state variables
  bh_->updateDerivedStateVars(&leoState_); // swing-stance switching happens here

  // construct new obs from CLeoState
  obs->resize(observation_dims_);
  bh_->parseLeoState(leoState_, *obs);
  bh_->updateDirection(test_?time_test_:time_learn_);

  bh_->setCurrentSTGState(NULL);
}

double LeoSquatEnvironment::step(const Vector &action, Vector *obs, double *reward, int *terminal)
{
  TRACE("RL action: " << action);

  Vector a = action;
  //a << -4, 5, -4;
  //std::cout << "RL action: " << action << std::endl;

  bh_->setCurrentSTGState(&leoState_);

  double actionArm = bh_->grlAutoActuateArm();
  if (a.size() == 6)
    target_action_ << actionArm, a[0], a[1], a[2], a[3], a[4], a[5];
  else
    target_action_ << actionArm, a[0], a[0], a[1], a[1], a[2], a[2];

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
  else if (bh_->isDoomedToFall(&leoState_, true))
    *terminal = 2;
  else
    *terminal = 0;

  LeoBaseEnvironment::step(tau, *reward, *terminal);

  // Update hip observations and squatting direction before the next timestep
  bh_->updateDirection(test_?time_test_:time_learn_);

  return tau;
}

void LeoSquatEnvironment::report(std::ostream &os)
{
  double trialTime  = test_?time_test_:time_learn_ - time0_;
  LeoBaseEnvironment::report(os);
  os << bh_->getProgressReport(trialTime);
}


