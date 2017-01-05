#include <XMLConfiguration.h>
#include <grl/environments/leo/leo_walk.h>

using namespace grl;

REGISTER_CONFIGURABLE(LeoBhWalkSym)
REGISTER_CONFIGURABLE(LeoBhWalk)
REGISTER_CONFIGURABLE(LeoWalkEnvironment)

double LeoBhWalk::calculateReward()
{
  // Original reward used by Erik
  return CLeoBhWalkSym::calculateReward();
}

void LeoBhWalk::parseStateFromTarget(const CLeoState &leoState, Vector &obs, const TargetInterface::ObserverInterface *observer) const
{
  int i, j;
  for (i = 0; i < observer->angles.size(); i++)
    obs[i] = leoState.mJointAngles[ observer->angles[i] ];
  for (j = 0; j < observer->angle_rates.size(); j++)
    obs[i+j] = leoState.mJointSpeeds[ observer->angle_rates[j] ];
  for (int k = 0; k < observer->augmented.size(); k++)
  {
    if (observer->augmented[k] == "heeltoe")
      obs[i+j+k] = (leoState.mFootContacts == 0?0:1); // any contact
    else if (observer->augmented[k] == "toeright")
      obs[i+j+k] = (leoState.mFootContacts & LEO_FOOTSENSOR_RIGHT_TOE);
    else if (observer->augmented[k] == "heelright")
      obs[i+j+k] = (leoState.mFootContacts & LEO_FOOTSENSOR_RIGHT_HEEL);
    else if (observer->augmented[k] == "toeleft")
      obs[i+j+k] = (leoState.mFootContacts & LEO_FOOTSENSOR_LEFT_TOE);
    else if (observer->augmented[k] == "heelleft")
      obs[i+j+k] = (leoState.mFootContacts & LEO_FOOTSENSOR_LEFT_HEEL);
    else
    {
      ERROR("Unknown augmented field '" << observer->augmented[i] << "'");
      throw bad_param("leo_walk:observer_idx_.augmented[i]");
    }
  }
}

void LeoBhWalk::parseActionForTarget(const Vector &action, Vector &target_action, const TargetInterface::ActuatorInterface *actuator)
{
    double actionArm, actionRightHip, actionLeftHip, actionRightKnee, actionLeftKnee, actionRightAnkle, actionLeftAnkle;

    if (actuator->voltage[avRightHipTorque] != -1)
      actionRightHip = action[ actuator->voltage[avRightHipTorque] ];
    if (actuator->voltage[avLeftHipTorque] != -1)
      actionLeftHip = action[ actuator->voltage[avLeftHipTorque] ];
    if (actuator->voltage[avRightKneeTorque] != -1)
      actionRightKnee = action[ actuator->voltage[avRightKneeTorque] ];
    if (actuator->voltage[avLeftKneeTorque] != -1)
      actionLeftKnee = action[ actuator->voltage[avLeftKneeTorque] ];
    if (actuator->voltage[avRightAnkleTorque] != -1)
      actionRightAnkle = action[ actuator->voltage[avRightAnkleTorque] ];
    if (actuator->voltage[avLeftAnkleTorque] != -1)
      actionLeftAnkle = action[ actuator->voltage[avLeftAnkleTorque] ];
    if (actuator->voltage[avLeftArmTorque] != -1)
      actionArm = action[ actuator->voltage[avLeftArmTorque] ];

    for (int i = 0; i < actuator->autoActuated.size(); i++)
    {
      if (actuator->autoActuated[i] == "shoulder")
        actionArm = grlAutoActuateArm();
      if (actuator->autoActuated[i] == "stanceknee")
      {
        // refers to an auto-actuation of the stance leg knee
        if (stanceLegLeft())
          actionLeftKnee = grlAutoActuateKnee();
        else
          actionRightKnee = grlAutoActuateKnee();
      }
       if (actuator->autoActuated[i] == "ankleright")
         grlAutoActuateRightAnkle(actionRightAnkle);
       if (actuator->autoActuated[i] == "ankleleft")
        grlAutoActuateLeftAnkle(actionLeftAnkle);
    }

    target_action << actionArm, actionRightHip, actionLeftHip, actionRightKnee, actionLeftKnee, actionRightAnkle, actionLeftAnkle;
}

std::string LeoBhWalk::getProgressReport(double trialTime)
{
  const int pw = 15;
  std::stringstream progressString;
  progressString << std::fixed << std::setprecision(3) << std::right;

  // Report from the base class
  progressString << CLeoBhWalkSym::getProgressReport(trialTime);

  // Number of footsteps in this trial
  progressString << std::setw(pw) << mNumFootsteps;

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

void LeoBhWalk::parseLeoState(const CLeoState &leoState, Vector &obs)
{
  parseStateFromTarget(leoState, obs, &interface_.observer);
}

void LeoBhWalk::parseLeoAction(const Vector &action, Vector &target_action)
{
  parseActionForTarget(action, target_action, &interface_.actuator);
}

/////////////////////////////////

void LeoBhWalkSym::parseLeoState(const CLeoState &leoState, Vector &obs)
{
  if (stanceLegLeft())
    parseStateFromTarget(leoState, obs, &interface_.observer);
  else
    parseStateFromTarget(leoState, obs, &interface_.observer_sym);
}

void LeoBhWalkSym::parseLeoAction(const Vector &action, Vector &target_action)
{
  if (stanceLegLeft())
    parseActionForTarget(action, target_action, &interface_.actuator);
  else
    parseActionForTarget(action, target_action, &interface_.actuator_sym);
}

/////////////////////////////////

LeoWalkEnvironment::LeoWalkEnvironment()
  : pub_ic_signal_(NULL)
{
}

void LeoWalkEnvironment::request(ConfigurationRequest *config)
{
  LeoBaseEnvironment::request(config);
  config->push_back(CRP("pub_ic_signal", "signal/vector", "Publisher of the initialization and contact signal", pub_ic_signal_, true));
}

void LeoWalkEnvironment::configure(Configuration &config)
{
  LeoBaseEnvironment::configure(config);
  pub_ic_signal_ = (VectorSignal*)config["pub_ic_signal"].ptr();

  // Augmenting state with a direction indicator variable, e.g: sit down or stand up
  const TargetInterface &interface = bh_->getInterface();
  Vector obs_min = config["observation_min"].v();
  Vector obs_max = config["observation_max"].v();

  int offset = interface.observer.angles.size()+interface.observer.angle_rates.size();// + interface.observer.contacts.size();
  for (int i = 0; i < interface.observer.augmented.size(); i++)
  {
    if (interface.observer.augmented[i] == "direction")
    {
      obs_min[offset + i] = -1;
      obs_max[offset + i] = +1;
    }
    else if ( (interface.observer.augmented[i] == "heeltoe") || // any contact
              (interface.observer.augmented[i] == "toeright") || (interface.observer.augmented[i] == "heelright") ||
              (interface.observer.augmented[i] == "toeleft") || (interface.observer.augmented[i] == "heelleft") )
    {
      obs_min[offset + i] =  0;
      obs_max[offset + i] =  1;
    }
    else
    {
      ERROR("Unknown augmented field '" << interface.observer.augmented[i] << "'");
      throw bad_param("leo_squat:os.augmented[i]");
    }
  }

  config.set("observation_min", obs_min);
  config.set("observation_max", obs_max);

  TRACE("Observation min: " << obs_min);
  TRACE("Observation max: " << obs_max);
}

LeoWalkEnvironment *LeoWalkEnvironment::clone() const
{
  return NULL;
}

void LeoWalkEnvironment::start(int test, Vector *obs)
{
  LeoBaseEnvironment::start(test);

  target_env_->start(test_, &target_obs_);

  // Parse obs into CLeoState (Start with left leg being the stance leg)
  bh_->fillLeoState(target_obs_, Vector(), leoState_);
  bh_->setCurrentSTGState(&leoState_);
  bh_->setPreviousSTGState(&leoState_);
  bh_->resetState(0);

  // update derived state variables
  bh_->updateDerivedStateVars(&leoState_); // swing-stance switching happens here

  // construct new obs from CLeoState
  obs->resize(observation_dims_);
  bh_->parseLeoState(leoState_, *obs);

  bh_->setCurrentSTGState(NULL);
}

double LeoWalkEnvironment::step(const Vector &action, Vector *obs, double *reward, int *terminal)
{
  CRAWL("RL action: " << action);

  bh_->setCurrentSTGState(&leoState_);

  // Reconstruct a Leo action from a possibly reduced agent action
  bh_->parseLeoAction(action, target_action_);
//  double auto_actuated_knee_voltage = bh_->grlAutoActuateKnee();

  // Execute action
  bh_->setPreviousSTGState(&leoState_);
  CRAWL(target_action_);
  double tau = target_env_->step(target_action_, &target_obs_, reward, terminal);
  CRAWL(target_obs_);

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

  // signal contact (agent may use this signal to tackle discontinuities)
  if (pub_ic_signal_)
  {
    if (!bh_->madeFootstep())
      pub_ic_signal_->set(VectorConstructor(lstNone));
    else
    {
      const TargetInterface ti = bh_->getInterface();
      Vector ti_actuator, ti_actuator_sym, signal;
      toVector(ti.actuator.voltage, ti_actuator);
      toVector(ti.actuator_sym.voltage, ti_actuator_sym);
      signal.resize(1+2*CLeoBhBase::svNumActions);
      // pack into a single vector
      if (bh_->stanceLegLeft())
        signal << VectorConstructor(lstContact), ti_actuator, ti_actuator_sym;
      else
        signal << VectorConstructor(lstContact), ti_actuator_sym, ti_actuator;
      pub_ic_signal_->set(signal);
    }
  }

  LeoBaseEnvironment::step(tau, *reward, *terminal);

  return tau;
}

void LeoWalkEnvironment::report(std::ostream &os) const
{
  double trialTime  = test_?time_test_:time_learn_ - time0_;
  os << bh_->getProgressReport(trialTime);
}

