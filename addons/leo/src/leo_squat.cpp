#include <XMLConfiguration.h>
#include <grl/environments/leo/leo_squat.h>

using namespace grl;

REGISTER_CONFIGURABLE(LeoSquatEnvironment)

double CLeoBhSquat::calculateReward()
{
  double reward = 0;

  // Negative reward for 'falling' (doomed to fall)
  if (isDoomedToFall(getCurrentSTGState(), false))
  {
    reward += mRwDoomedToFall;
    mLogDebugLn("[REWARD] Doomed to fall! Reward: " << mRwDoomedToFall << " (total reward: " << getTotalReward() << ")" << endl);
  }

  // Reward for keeping torso upright
  double torsoReward = mRwTorsoUpright * 1.0/(1.0 + (getCurrentSTGState()->mJointAngles[ljTorso] - mRwTorsoUprightAngle)*(getCurrentSTGState()->mJointAngles[ljTorso] - mRwTorsoUprightAngle)/(mRwTorsoUprightAngleMargin*mRwTorsoUprightAngleMargin));
  reward += torsoReward;

  return reward;
}

/////////////////////////////////

LeoSquatEnvironment::LeoSquatEnvironment() :
  requested_action_dims_(CLeoBhBase::svNumActions)
{
  bh_ = new CLeoBhSquat(&leoSim_);
}

void LeoSquatEnvironment::request(ConfigurationRequest *config)
{
  LeoBaseEnvironment::request(config);
}

void LeoSquatEnvironment::configure(Configuration &config)
{
  LeoBaseEnvironment::configure(config);
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
