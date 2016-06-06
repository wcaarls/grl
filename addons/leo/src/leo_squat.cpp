#include <XMLConfiguration.h>
#include <grl/environments/leo/leo_squat.h>

using namespace grl;

REGISTER_CONFIGURABLE(LeoSquatEnvironment)

const double T[] = { 0.0,  0.0,  0.0,  0.0,  0.0};
const double B[] = { 1.4,  1.4, -1.9, -1.9, -0.3}; // hips, knees, torso

double CLeoBhSquat::calculateReward()
{
  double reward = 0;
  CLeoState *s = getCurrentSTGState();

  // Negative reward for 'falling' (doomed to fall)
  if (isDoomedToFall(s, false))
  {
    reward += mRwDoomedToFall;
    mLogDebugLn("[REWARD] Doomed to fall! Reward: " << mRwDoomedToFall << " (total reward: " << getTotalReward() << ")" << endl);
  }
  else
  {
    double rw;
    if (direction_ == -1)
      rw = r(s->mJointAngles, B);
    else if (direction_ == 1)
      rw = r(s->mJointAngles, T);
    reward += -0.1*rw;

    if (direction_ != prev_direction_)
      reward += 50;

    // Reward for keeping torso upright
    //double torsoReward = mRwTorsoUpright * 1.0/(1.0 + (s->mJointAngles[ljTorso] - mRwTorsoUprightAngle)*(s->mJointAngles[ljTorso] - mRwTorsoUprightAngle)/(mRwTorsoUprightAngleMargin*mRwTorsoUprightAngleMargin));
    //reward += torsoReward;
  }

  return reward;
}

double CLeoBhSquat::r(const double *x, const double* y) const
{
  return  pow(fabs(x[ljHipLeft]   - y[0]), 2) +
          pow(fabs(x[ljHipRight]  - y[1]), 2) +
          pow(fabs(x[ljKneeLeft]  - y[2]), 2) +
          pow(fabs(x[ljKneeRight] - y[3]), 2) +
          pow(fabs(x[ljTorso]     - y[4]), 2);
}

bool CLeoBhSquat::isSitting(const double *x) const
{
  double eps = 0.1;
  if ( (fabs(x[ljHipLeft]   - B[0]) < eps) &&
       (fabs(x[ljHipRight]  - B[1]) < eps) &&
       (fabs(x[ljKneeLeft]  - B[2]) < eps) &&
       (fabs(x[ljKneeRight] - B[3]) < eps) &&
       (fabs(x[ljTorso]     - B[4]) < eps) )
    return true;
  return false;
}

bool CLeoBhSquat::isStanding(const double *x) const
{
  double eps = 0.1;
  if ( (fabs(x[ljHipLeft]   - T[0]) < eps) &&
       (fabs(x[ljHipRight]  - T[1]) < eps) &&
       (fabs(x[ljKneeLeft]  - T[2]) < eps) &&
       (fabs(x[ljKneeRight] - T[3]) < eps) &&
       (fabs(x[ljTorso]     - T[4]) < eps) )
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
  prev_direction_ = direction_;
  if (direction_ == -1 && isSitting(obs.data()))
    obs[osDirection] = direction_ =  1;
  else if (direction_ == 1 && isStanding(obs.data()))
    obs[osDirection] = direction_ = -1;
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

  // Determine reward
  *reward = bh_->calculateReward();

  // ... and termination
  if (*terminal == 1) // timeout
    *terminal = 1;
  else if (bh_->isDoomedToFall(&leoState_, false))
    *terminal = 2;
  else
    *terminal = 0;

  // construct new obs from CLeoState
  bh_->parseLeoState(leoState_, *obs);

  LeoBaseEnvironment::step(tau, *reward, *terminal);
  return tau;
}

