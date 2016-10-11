#include <XMLConfiguration.h>
#include <grl/environments/leo/leo_walk.h>

using namespace grl;

REGISTER_CONFIGURABLE(LeoWalkEnvironment)

double CLeoBhWalk::calculateReward()
{
  // Original reward used by Erik
  return CLeoBhWalkSym::calculateReward();
}

void CLeoBhWalk::parseLeoState(const CLeoState &leoState, Vector &obs)
{
  EnvironmentAgentInterface::ObserverInterface *observer;

  if (stanceLegLeft())
    observer = &(interface_.observer);
  else
    observer = &(interface_.observer_sym);

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

void CLeoBhWalk::parseLeoAction(const Vector &action, Vector &target_action)
{
  double actionArm, actionStanceHip, actionSwingHip, actionStanceKnee, actionSwingKnee, actionStanceAnkle, actionSwingAnkle;

  // Right leg is assumed to be a stance leg in the configuration file
  if (interface_.actuator.actions[avRightHipTorque] != -1)
    actionStanceHip = action[ interface_.actuator.actions[avRightHipTorque] ];
  if (interface_.actuator.actions[avLeftHipTorque] != -1)
    actionSwingHip = action[ interface_.actuator.actions[avLeftHipTorque] ];
  if (interface_.actuator.actions[avRightKneeTorque] != -1)
    actionStanceKnee = action[ interface_.actuator.actions[avRightKneeTorque] ];
  if (interface_.actuator.actions[avLeftKneeTorque] != -1)
    actionSwingKnee = action[ interface_.actuator.actions[avLeftKneeTorque] ];
  if (interface_.actuator.actions[avRightAnkleTorque] != -1)
    actionStanceAnkle = action[ interface_.actuator.actions[avRightAnkleTorque] ];
  if (interface_.actuator.actions[avLeftAnkleTorque] != -1)
    actionSwingAnkle = action[ interface_.actuator.actions[avLeftAnkleTorque] ];

  bool ankle_autoActuated = false;
  for (int i = 0; i < interface_.actuator.autoActuated.size(); i++)
  {
    if (interface_.actuator.autoActuated[i] == "shoulder")
      actionArm = grlAutoActuateArm();
    if (interface_.actuator.autoActuated[i] == "kneeright") // refers to a stance leg
      actionStanceKnee = grlAutoActuateKnee();
    if ((interface_.actuator.autoActuated[i] == "ankleright") || (interface_.actuator.autoActuated[i] == "ankleleft"))
    {
      if (!ankle_autoActuated)
      {
        grlAutoActuateAnkles(actionStanceAnkle, actionSwingAnkle);
        ankle_autoActuated = true;
      }
    }
  }

  // concatenation happens in the order of <actionvar> definitions in an xml file
  // shoulder, right hip, left hip, right knee, left knee, right ankle, left ankle
  if (stanceLegLeft())
    target_action << actionArm, actionSwingHip, actionStanceHip, actionSwingKnee, actionStanceKnee, actionSwingAnkle, actionStanceAnkle;
  else
    target_action << actionArm, actionStanceHip, actionSwingHip, actionStanceKnee, actionSwingKnee, actionStanceAnkle, actionSwingAnkle;

  //ode_action_ << ConstantVector(7, 5.0); // #ivan
}


std::string CLeoBhWalk::getProgressReport(double trialTime)
{
  const int pw = 15;
  std::stringstream progressString;
  progressString << std::fixed << std::setprecision(3) << std::right;

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

/////////////////////////////////

LeoWalkEnvironment::LeoWalkEnvironment()
  : mirror_sig_(NULL)
{
  bh_ = new CLeoBhWalk(&leoSim_);
  set_bh(bh_);
}

void LeoWalkEnvironment::request(ConfigurationRequest *config)
{
  LeoBaseEnvironment::request(config);
  config->push_back(CRP("contact_signal", "signal", "Contact signal", mirror_sig_, true));
}

void LeoWalkEnvironment::configure(Configuration &config)
{
  LeoBaseEnvironment::configure(config);
  mirror_sig_ = (Signal*)config["contact_signal"].ptr();

  // Augmenting state with a direction indicator variable: sit down or stand up
  const EnvironmentAgentInterface &interface = bh_->getInterface();
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
  bh_->resetState();

  // update derived state variables
  bh_->updateDerivedStateVars(&leoState_); // swing-stance switching happens here

  // construct new obs from CLeoState
  obs->resize(observation_dims_);
  bh_->parseLeoState(leoState_, *obs);

  bh_->setCurrentSTGState(NULL);

  // signal contact (agent may use this signal to tackle discontinuities)
  if (mirror_sig_)
    mirror_sig_->set(VectorConstructor(0, 0, 0, 0, 0, 1)); // last '1' means start => any action is possible
}

double LeoWalkEnvironment::step(const Vector &action, Vector *obs, double *reward, int *terminal)
{
  TRACE("RL action: " << action);

  bh_->setCurrentSTGState(&leoState_);

  // Reconstruct a Leo action from a possibly reduced agent action
  bh_->parseLeoAction(action, target_action_);
  double auto_actuated_knee_voltage = bh_->grlAutoActuateKnee();

  // Execute action
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

  // signal contact (agent may use this signal to tackle discontinuities)
  if (mirror_sig_)
  {
    mirror_sig_->set(VectorConstructor(bh_->madeFootstep(), auto_actuated_knee_voltage, 0, 1, 2, 0)); // last '0' means step
    TRACE(auto_actuated_knee_voltage);
    if (bh_->madeFootstep())
      std::cout << "contact happened" << std::endl;
  }

  LeoBaseEnvironment::step(tau, *reward, *terminal);

  return tau;
}

void LeoWalkEnvironment::report(std::ostream &os) const
{
  double trialTime  = test_?time_test_:time_learn_ - time0_;
  LeoBaseEnvironment::report(os);
  os << bh_->getProgressReport(trialTime);
}

