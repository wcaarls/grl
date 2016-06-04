#include <XMLConfiguration.h>
#include <grl/environments/leo/leo_squat.h>

using namespace grl;

REGISTER_CONFIGURABLE(LeoSquatEnvironment)

double CLeoBhSquat::calculateReward()
{
  return CLeoBhWalkSym::calculateReward();
}

/////////////////////////////////

LeoSquatEnvironment::LeoSquatEnvironment() :
  requested_action_dims_(CLeoBhBase::svNumActions),
  learn_stance_knee_(0)
{
  bh_ = new CLeoBhSquat(&leoSim_);
}

void LeoSquatEnvironment::request(ConfigurationRequest *config)
{
  LeoBaseEnvironment::request(config);
  config->push_back(CRP("learn_stance_knee", "Learn stance knee", learn_stance_knee_, CRP::Configuration, 0, 1));
}

void LeoSquatEnvironment::configure(Configuration &config)
{
  LeoBaseEnvironment::configure(config);

  config_parse_observations(config);
  config_parse_actions(config);       // Parse actions specifically for walking
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

  // auto actuate unlearned joints to find complete action vector
  double actionArm, actionStanceKnee, actionSwingKnee, actionStanceHip, actionSwingHip;
  actionStanceHip = action[0];
  actionSwingHip  = action[1];
  if (!learn_stance_knee_)
  {
    // Auto actuation of the stance knee
    actionStanceKnee = bh_->grlAutoActuateKnee();
    actionSwingKnee  = action[2];
  }
  else
  {
    // Learn both actions
    actionStanceKnee = action[2];
    actionSwingKnee  = action[3];
  }
  Vector actionAnkles;
  bh_->grlAutoActuateAnkles(actionAnkles);
  actionArm = bh_->grlAutoActuateArm();

  // concatenation happens in the order of <actionvar> definitions in an xml file
  // shoulder, right hip, left hip, right knee, left knee, right ankle, left ankle
  if (bh_->stanceLegLeft())
    target_action_ << actionArm, actionSwingHip, actionStanceHip, actionSwingKnee, actionStanceKnee, actionAnkles;
  else
    target_action_ << actionArm, actionStanceHip, actionSwingHip, actionStanceKnee, actionSwingKnee, actionAnkles;

  //ode_action_ << ConstantVector(7, 5.0); // #ivan

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

void LeoSquatEnvironment::config_parse_actions(Configuration &config)
{
  std::vector<int> knee_idx;
  std::string knee = std::string("knee");
  int omit_knee_idx = -1;
  std::string actuate = config["actuate"].str();
  std::vector<std::string> actuateList = cutLongStr(actuate);
  fillActuate(ode_->getActuators(), actuateList, actuate_, &knee, &knee_idx);
  if (actuate_.size() != target_action_dims_)
    throw bad_param("leosim/walk:actuate");
  if (knee_idx.size() == 0)
    throw bad_param("leosim/walk:knee");
  requested_action_dims_ = (actuate_.array() != 0).count();
  if (learn_stance_knee_)
    action_dims_ = requested_action_dims_;
  else
  {
    if (knee_idx.size() != 2)
      throw bad_param("leosim/walk:actuate (if any of knees is learnt, then always include both knees)");
    action_dims_ = requested_action_dims_ - 1;
    omit_knee_idx = knee_idx[1];
  }

  // mask observation min/max vectors
  Vector ode_action_min, ode_action_max, action_min, action_max;
  config.get("action_min", ode_action_min);
  config.get("action_max", ode_action_max);
  action_min.resize(action_dims_);
  action_max.resize(action_dims_);
  for (int i = 0, j = 0; i < actuate_.size(); i++)
    if (actuate_[i] && i != omit_knee_idx)
    {
      action_min[j]   = ode_action_min[i];
      action_max[j++] = ode_action_max[i];
    }

  config.set("action_dims", action_dims_);
  config.set("action_min", action_min);
  config.set("action_max", action_max);

  // reserve memory
  target_obs_.resize(target_observation_dims_);
  target_action_.resize(target_action_dims_);
}
