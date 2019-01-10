#include <sys/stat.h>
#include <XMLConfiguration.h>
#include <grl/environments/leo/leo.h>

using namespace grl;

void CLeoBhBase::resetState(double time0)
{
  mIsObserving          = false;
  mLastRewardedFoot     = lpFootLeft;
  mLastStancelegWasLeft = -1;
  mFootstepLength       = 0.0;
  mLastFootstepLength   = 0.0;
  mNumFootsteps         = 0;
  mWalkedDistance       = 0.0;
  mTrialEnergy          = 0.0;

  // Reset velocity filters to zero velocity (this is the result of robot->setIC)
  for (int iJoint=0; iJoint<ljNumJoints; iJoint++)
    mJointSpeedFilter[iJoint].clear();

  for (int i=0; i<ljNumDynamixels; i++)
    mJointSpeedFilter[i].init(1.0/mTotalStepTime, 10.0);
  mJointSpeedFilter[ljTorso].init(mTotalStepTime, 25.0);	// 25Hz because? : 1) this encoder has 8x the resolution of a dynamixel 2) torso angles/velocities are more important
}

void CLeoBhBase::fillLeoState(const Vector &obs, const Vector &action, CLeoState &leoState)
{
  // '-' required to match with Erik's code, but does not matter for learning.
  // Erik used a rotation matrix which was rotating a unit vector. For torso it seems
  // the positive direction was not same as for other joints, internally defined in ODE.
  leoState.mJointAngles[ljTorso]      = -obs[svTorsoAngle];
  leoState.mJointSpeeds[ljTorso]      = -mJointSpeedFilter[ljTorso].filter(obs[svTorsoAngleRate]);
  leoState.mJointAngles[ljShoulder]   = obs[svLeftArmAngle];
  leoState.mJointSpeeds[ljShoulder]   = mJointSpeedFilter[ljShoulder].filter(obs[svLeftArmAngleRate]);
  leoState.mJointAngles[ljHipRight]   = obs[svRightHipAngle];
  leoState.mJointSpeeds[ljHipRight]   = mJointSpeedFilter[ljHipRight].filter(obs[svRightHipAngleRate]);
  leoState.mJointAngles[ljHipLeft]    = obs[svLeftHipAngle];
  leoState.mJointSpeeds[ljHipLeft]    = mJointSpeedFilter[ljHipLeft].filter(obs[svLeftHipAngleRate]);
  leoState.mJointAngles[ljKneeRight]  = obs[svRightKneeAngle];
  leoState.mJointSpeeds[ljKneeRight]	= mJointSpeedFilter[ljKneeRight].filter(obs[svRightKneeAngleRate]);
  leoState.mJointAngles[ljKneeLeft]   = obs[svLeftKneeAngle];
  leoState.mJointSpeeds[ljKneeLeft]   = mJointSpeedFilter[ljKneeLeft].filter(obs[svLeftKneeAngleRate]);
  leoState.mJointAngles[ljAnkleRight] = obs[svRightAnkleAngle];
  leoState.mJointSpeeds[ljAnkleRight] = mJointSpeedFilter[ljAnkleRight].filter(obs[svRightAnkleAngleRate]);
  leoState.mJointAngles[ljAnkleLeft]  = obs[svLeftAnkleAngle];
  leoState.mJointSpeeds[ljAnkleLeft]	= mJointSpeedFilter[ljAnkleLeft].filter(obs[svLeftAnkleAngleRate]);

  leoState.mFootContacts  = obs[svRightToeContact] > 0.5  ? LEO_FOOTSENSOR_RIGHT_TOE  : 0;
  leoState.mFootContacts |= obs[svRightHeelContact] > 0.5 ? LEO_FOOTSENSOR_RIGHT_HEEL : 0;
  leoState.mFootContacts |= obs[svLeftToeContact] > 0.5  ? LEO_FOOTSENSOR_LEFT_TOE   : 0;
  leoState.mFootContacts |= obs[svLeftHeelContact] >0.5 ? LEO_FOOTSENSOR_LEFT_HEEL  : 0;

  // required for the correct energy calculation in the reward function
  if (action.size())
  {
    if (interface_.actuator.mode == amVoltage)
    {
      leoState.mActuationVoltages[ljShoulder]   = action[avLeftArmAction];
      leoState.mActuationVoltages[ljHipRight]   = action[avRightHipAction];
      leoState.mActuationVoltages[ljHipLeft]    = action[avLeftHipAction];
      leoState.mActuationVoltages[ljKneeRight]  = action[avRightKneeAction];
      leoState.mActuationVoltages[ljKneeLeft]   = action[avLeftKneeAction];
      leoState.mActuationVoltages[ljAnkleRight] = action[avRightAnkleAction];
      leoState.mActuationVoltages[ljAnkleLeft]  = action[avLeftAnkleAction];
    }
    else if (interface_.actuator.mode == amTorque)
    {
      leoState.mActuationTorques[ljShoulder]   = action[avLeftArmAction];
      leoState.mActuationTorques[ljHipRight]   = action[avRightHipAction];
      leoState.mActuationTorques[ljHipLeft]    = action[avLeftHipAction];
      leoState.mActuationTorques[ljKneeRight]  = action[avRightKneeAction];
      leoState.mActuationTorques[ljKneeLeft]   = action[avLeftKneeAction];
      leoState.mActuationTorques[ljAnkleRight] = action[avRightAnkleAction];
      leoState.mActuationTorques[ljAnkleLeft]  = action[avLeftAnkleAction];
    }
  }
}

void CLeoBhBase::parseLeoState(const CLeoState &leoState, Vector &obs)
{
  obs[owTorsoAngle]           = leoState.mJointAngles[ljTorso];
  obs[owTorsoAngleRate]       = leoState.mJointSpeeds[ljTorso];
  obs[owHipStanceAngle]       = leoState.mJointAngles[mHipStance];
  obs[owHipStanceAngleRate]   = leoState.mJointSpeeds[mHipStance];
  obs[owHipSwingAngle]        = leoState.mJointAngles[mHipSwing];
  obs[owHipSwingAngleRate]    = leoState.mJointSpeeds[mHipSwing];
  obs[owKneeStanceAngle]      = leoState.mJointAngles[mKneeStance];
  obs[owKneeStanceAngleRate]  = leoState.mJointSpeeds[mKneeStance];
  obs[owKneeSwingAngle]       = leoState.mJointAngles[mKneeSwing];
  obs[owKneeSwingAngleRate]   = leoState.mJointSpeeds[mKneeSwing];
}

void CLeoBhBase::setCurrentSTGState(CLeoState *leoState)
{
  mCurrentSTGState = leoState;
}

void CLeoBhBase::setPreviousSTGState(CLeoState *leoState)
{
  mPreviousSTGState = *leoState;
}

void CLeoBhBase::updateDerivedStateVars(CLeoState* currentSTGState)
{
  CLeoBhWalkSym::updateDerivedStateVars(currentSTGState);
}

bool CLeoBhBase::madeFootstep()
{
  return CLeoBhWalkSym::mMadeFootstep;
}

int CLeoBhBase::jointNameToIndex(const std::string jointName) const
{
  if (jointName == "torso_boom")
    return ljTorso;
  else if (jointName == "shoulder")
    return ljShoulder;
  else if (jointName == "hipright")
    return ljHipRight;
  else if (jointName == "hipleft")
    return ljHipLeft;
  else if (jointName == "kneeright")
    return ljKneeRight;
  else if (jointName == "kneeleft")
    return ljKneeLeft;
  else if (jointName == "ankleright")
    return ljAnkleRight;
  else if (jointName == "ankleleft")
    return ljAnkleLeft;
  else
    return -1; // augmented state
}

std::string CLeoBhBase::jointIndexToName(int jointIndex) const
{
  switch(jointIndex)
  {
    case ljTorso      : return std::string("torso_boom"); break;
    case ljShoulder   : return std::string("shoulder");   break;
    case ljHipRight   : return std::string("hipright");   break;
    case ljHipLeft    : return std::string("hipleft");    break;
    case ljKneeRight  : return std::string("kneeright");  break;
    case ljKneeLeft   : return std::string("kneeleft");   break;
    case ljAnkleRight : return std::string("ankleright"); break;
    case ljAnkleLeft  : return std::string("ankleleft");  break;
    default:
      ERROR("Joint index out of bounds '" << jointIndex << "'");
      throw bad_param("leobase:jointIndex");
  }
}

/////////////////////////////////

LeoBaseEnvironment::LeoBaseEnvironment() :
  target_env_(NULL),
  bh_(NULL),
  observation_dims_(CLeoBhBase::svNumStates),
  action_dims_(CLeoBhBase::svNumActions),
  pub_ic_signal_(NULL),
  exporter_(NULL),
  test_(0),
  time_test_(0),
  time_learn_(0),
  time0_(0),
  measurement_noise_(0),
  sub_transition_type_(NULL)
{
}

void LeoBaseEnvironment::request(ConfigurationRequest *config)
{
  config->push_back(CRP("behavior", "behavior", "Behavior type", bh_));

  config->push_back(CRP("xml", "XML configuration filename", xml_));
  config->push_back(CRP("target_env", "environment", "Interaction environment", target_env_));
  config->push_back(CRP("observe", "Comma-separated list of state elements observed by an agent", ""));
  config->push_back(CRP("actuate", "Comma-separated list of action elements provided by an agent", ""));
  config->push_back(CRP("exporter", "exporter", "Optional exporter for transition log (supports time, state, observation, action, reward, terminal)", exporter_, true));
  config->push_back(CRP("sub_transition_type", "signal/vector", "Subscriber to the transition type", sub_transition_type_, true));
  config->push_back(CRP("pub_ic_signal", "signal/vector", "Publisher of the initialization and contact signal", pub_ic_signal_, true));
  config->push_back(CRP("measurement_noise", "Additive measurement noise", (double)measurement_noise_, CRP::Configuration));

  config->push_back(CRP("observation_dims", "int.observation_dims", "Number of observation dimensions", CRP::Provided));
  config->push_back(CRP("observation_min", "vector.observation_min", "Lower limit on observations", CRP::Provided));
  config->push_back(CRP("observation_max", "vector.observation_max", "Upper limit on observations", CRP::Provided));
  config->push_back(CRP("action_dims", "int.action_dims", "Number of action dimensions", CRP::Provided));
  config->push_back(CRP("action_min", "vector.action_min", "Lower limit on actions", CRP::Provided));
  config->push_back(CRP("action_max", "vector.action_max", "Upper limit on actions", CRP::Provided));
}

void LeoBaseEnvironment::configure(Configuration &config)
{
  // here we can select an actual Leo enviromnent (simulation/real)
  target_env_ = (Environment*)config["target_env"].ptr();

  bh_ = (CLeoBhBase*)config["behavior"].ptr();
  sub_transition_type_ = (VectorSignal*)config["sub_transition_type"].ptr();
  pub_ic_signal_ = (VectorSignal*)config["pub_ic_signal"].ptr();
  measurement_noise_ = config["measurement_noise"];

  // Setup path to a configuration file
  xml_ = config["xml"].str();
  struct stat buffer;
  if (stat (xml_.c_str(), &buffer) != 0)
    xml_ = std::string(LEO_CONFIG_DIR) + "/" + config["xml"].str();
  std::cout << xml_ << std::endl;

  exporter_ = (Exporter*) config["exporter"].ptr();
  if (exporter_)
    exporter_->init({"time", "state0", "state1", "action", "reward", "terminal", "transition_type", "contact"});

  // Process configuration of Leo
  CXMLConfiguration xmlConfig;
  if (!xmlConfig.loadFile(xml_))
  {
    ERROR("Couldn't load XML configuration file \"" << xml_ << "\"!\nPlease check that the file exists and that it is sound (error: " << xmlConfig.errorStr() << ").");
    return;
  }

  // Resolve expressions
  xmlConfig.resolveExpressions();

  // Read rewards and preprogrammed angles
  bh_->readConfig(xmlConfig.root());

  // Create ode object which resolves states and actions
  ODESTGEnvironment *ode = new ODESTGEnvironment();
  if (!ode->configure(config))
  {
    ERROR("Could not initialize STG ODE environment");
    return;
  }

  // Select states and actions that are delivered to an agent
  configParseObservations(config, ode->getSensors());
  configParseActions(config, ode->getActuators());

  target_obs_.v.resize(ode->getSensors().size());
  target_action_.v.resize(ode->getActuators().size());

  delete ode;
}

void LeoBaseEnvironment::reconfigure(const Configuration &config)
{
  time_test_ = time_learn_ = time0_ = 0;
}

void LeoBaseEnvironment::start(int test)
{
  test_ = test;
  bh_->resetState(0);

  if (exporter_)
    exporter_->open((test_?"test":"learn"), (test_?time_test_:time_learn_) != 0.0);
  time0_ = test_?time_test_:time_learn_;
}

void LeoBaseEnvironment::step(double tau, double reward, int terminal)
{
  double &time = test_?time_test_:time_learn_;

  // Export & debug
  std::vector<double> s1(leoState_.mJointAngles, leoState_.mJointAngles + ljNumJoints);
  std::vector<double> v1(leoState_.mJointSpeeds, leoState_.mJointSpeeds + ljNumJoints);
  std::vector<double> a;
  if (bh_->getActuationInterface()->getActuationMode() == amVoltage)
    std::copy(&(leoState_.mActuationVoltages[0]), &(leoState_.mActuationVoltages[ljNumDynamixels]), std::back_inserter(a));
  else if (bh_->getActuationInterface()->getActuationMode() == amTorque)
    std::copy(&(leoState_.mActuationTorques[0]), &(leoState_.mActuationTorques[ljNumDynamixels]), std::back_inserter(a));

  CRAWL("State angles: " << s1);
  CRAWL("State velocities: " << v1);
  CRAWL("Contacts: " << (int)leoState_.mFootContacts);
  CRAWL("Full action: " << a);
  CRAWL("Reward: " << reward);

  if (exporter_)
  {
    // transition type
    Vector tt, ic;
    if (sub_transition_type_)
      tt = sub_transition_type_->get();
    if (pub_ic_signal_)
      ic = VectorConstructor(pub_ic_signal_->get()[0]);

    std::vector<double> s0(bh_->getPreviousSTGState()->mJointAngles, bh_->getPreviousSTGState()->mJointAngles + ljNumJoints);
    std::vector<double> v0(bh_->getPreviousSTGState()->mJointSpeeds, bh_->getPreviousSTGState()->mJointSpeeds + ljNumJoints);
    s0.insert(s0.end(), v0.begin(), v0.end());
    s1.insert(s1.end(), v1.begin(), v1.end());

    Vector s0v, s1v, av;
    toVector(s0, s0v);
    toVector(s1, s1v);
    toVector(a, av);

    exporter_->write({grl::VectorConstructor(time), s0v,  s1v,
                      av, grl::VectorConstructor(reward), grl::VectorConstructor(terminal), tt, ic
                     });
  }

  time += tau;
}

void LeoBaseEnvironment::report(std::ostream &os) const
{
  double trialTime  = test_?time_test_:time_learn_ - time0_;
  os << bh_->getProgressReport(trialTime);
}

///////////////////////////////////////////
/// Helper functions
///
void LeoBaseEnvironment::configParseObservations(Configuration &config, const std::vector<CGenericStateVar> &sensors)
{
  const std::vector<std::string> observeList = cutLongStr( config["observe"].str() );
  std::vector<std::string> observed;
  fillObserve(sensors, observeList, observed);
  TargetInterface::ObserverInterface int_observer;
  fillObserver(observed, int_observer);
  observation_dims_ = observed.size();

  // mirror left and right legs
  std::vector<std::string> observed_sym = observed;
  for (int i = 0; i < observed.size(); i++)
  {
    std::size_t idx = observed[i].find("right");
    if (idx != std::string::npos)
      observed_sym[i].replace(idx, 5, "left");
    idx = observed[i].find("left");
    if (idx != std::string::npos)
      observed_sym[i].replace(idx, 4, "right");
  }
  TargetInterface::ObserverInterface int_observer_sym;
  fillObserver(observed_sym, int_observer_sym);

  // mask observation min/max vectors
  Vector target_observation_min, target_observation_max, observation_min, observation_max;
  config.get("observation_min", target_observation_min);
  config.get("observation_max", target_observation_max);
  observation_min.resize(observation_dims_);
  observation_max.resize(observation_dims_);

  int i, j;
  for (i = 0; i < int_observer.angles.size(); i++)
  {
    std::string name = "robot." + bh_->jointIndexToName(int_observer.angles[i]) + ".angle";
    int sensor_idx = findVarIdx(sensors, name);
    observation_min[i] = target_observation_min[sensor_idx];
    observation_max[i] = target_observation_max[sensor_idx];
  }
  for (j = 0; j < int_observer.angle_rates.size(); j++)
  {
    std::string name = "robot." + bh_->jointIndexToName(int_observer.angle_rates[j]) + ".anglerate";
    int sensor_idx = findVarIdx(sensors, name);
    observation_min[i+j] = target_observation_min[sensor_idx];
    observation_max[i+j] = target_observation_max[sensor_idx];
  }

  // Set parameters exported to an agent
  config.set("observation_dims", observation_dims_);
  config.set("observation_min", observation_min);
  config.set("observation_max", observation_max);

  // Prepare observer indexes for easy connection between states of the target environment and agent observations
  bh_->setObserverInterface(int_observer, int_observer_sym);
}

void LeoBaseEnvironment::ensure_bounds(Vector *action) const
{
  // value function without bounds leosim_sarsa_walk_egreedy-run0-nb-_experiment_agent_policy_representation.dat
  for (int i = 0; i < action->size(); i++)
  {
    // first apply formal bounds on action
    (*action)[i] = fmin(fmax((*action)[i], target_action_min_[i]), target_action_max_[i]);

    if (bh_->getActuationMode() == amTorque)
    {
      // second apply Power Supply bound (we cannot supply voltages higher then power supply)
      double omega;
      if (bh_->getPreviousSTGState()->isValid())  // We don't have a previous state at the beginning of a trial
        omega = 0.5*(bh_->getCurrentSTGState()->mJointSpeeds[i] + bh_->getPreviousSTGState()->mJointSpeeds[i]);
      else
        omega = bh_->getCurrentSTGState()->mJointSpeeds[i];

      double max_torque = fabs(DXL_TORQUE_CONST*DXL_GEARBOX_RATIO*(LEO_MAX_DXL_VOLTAGE-DXL_TORQUE_CONST*DXL_GEARBOX_RATIO*omega)/DXL_RESISTANCE);
/*
      if (fabs((*action)[i]) > max_torque)
      {
        double I = (*action)[i] / (DXL_TORQUE_CONST*DXL_GEARBOX_RATIO);
        double U = I*DXL_RESISTANCE + DXL_TORQUE_CONST*DXL_GEARBOX_RATIO*omega;
        std::cout << "[ensure_bounds] Voltage " << U << " exeeded maximum value;" <<std::endl;
      }
*/
      (*action)[i] = fmin(fmax((*action)[i], -max_torque), max_torque);
    }
  }
}

void LeoBaseEnvironment::add_measurement_noise(Vector *state) const
{
  Rand rd;
  for (size_t ii=0; ii < state->size(); ++ii)
    (*state)[ii] += rd.getUniform(-measurement_noise_, measurement_noise_);
}

int LeoBaseEnvironment::findVarIdx(const std::vector<CGenericStateVar> &genericStates, std::string query) const
{
  std::vector<CGenericStateVar>::const_iterator gState = genericStates.begin();
  for (int i = 0; gState < genericStates.end(); gState++, i++)
    if (query == gState->name())
      return i;
  return -1;
}

void LeoBaseEnvironment::fillObserver(const std::vector<std::string> &observed, TargetInterface::ObserverInterface &observer_interface) const
{
  for (int i = 0; i < observed.size(); i++)
  {
    std::string name = observed[i];
    std::replace( name.begin(), name.end(), '.', ' ');
    std::vector<std::string> cuttedName = cutLongStr(name);
    if (cuttedName.size() == 1)
      observer_interface.augmented.push_back(name);
    else if (cuttedName[2] == "angle")
      observer_interface.angles.push_back(bh_->jointNameToIndex(cuttedName[1]));
    else if (cuttedName[2] == "anglerate")
      observer_interface.angle_rates.push_back(bh_->jointNameToIndex(cuttedName[1]));
    else if (cuttedName[2] == "contact")
      observer_interface.augmented.push_back(cuttedName[1]);
    else
    {
      ERROR("Unknown joint '" << cuttedName[2] << "'");
      throw bad_param("leobase:fillObserver:cuttedName[2]");
    }
  }
}

void LeoBaseEnvironment::configParseActions(Configuration &config, const std::vector<CGenericActionVar> &actuators)
{
  TargetInterface::ActuatorInterface int_actuator, int_actuator_sym;
  std::vector<std::string> actuateList = cutLongStr(config["actuate"].str());
  fillActuate(actuators, actuateList, int_actuator);
  TRACE("Actuate '" << int_actuator.action << "'"); // array which maps target_environment action vector to an agent action vector
  action_dims_ = actuateList.size();

  // mirror left and right legs
  INFO("Symmetrically map actuations");
  std::vector<std::string> actuateList_sym = actuateList;
  for (int i = 0; i < actuateList.size(); i++)
  {
    std::size_t idx = actuateList[i].find("right");
    if (idx != std::string::npos)
      actuateList_sym[i].replace(idx, 5, "left");
    idx = actuateList[i].find("left");
    if (idx != std::string::npos)
      actuateList_sym[i].replace(idx, 4, "right");
  }
  fillActuate(actuators, actuateList_sym, int_actuator_sym);

  // mask observation min/max vectors
  Vector action_min, action_max;
  config.get("action_min", target_action_min_);
  config.get("action_max", target_action_max_);
  action_min.resize(action_dims_);
  action_max.resize(action_dims_);

  for (int j = 0; j < action_dims_; j++)
  {
    // if we actuate a few joints by the same value, as "knee" from 'actuateList' will actuate both left and right knees
    double min = -std::numeric_limits<double>::max();
    double max = +std::numeric_limits<double>::max();
    for (int i = 0; i < int_actuator.action.size(); i++)
    {
      if (int_actuator.action[i] == j)
      {
        max = fmin(target_action_max_[i], max);
        min = fmax(target_action_min_[i], min);
      }
    }

    // actuate required joints but narrowest bound is enforced
    action_min[j] = min;
    action_max[j] = max;
  }

  // Set parameters exported to an agent
  config.set("action_dims", action_dims_);
  config.set("action_min", action_min);
  config.set("action_max", action_max);

  TRACE("Action min: " << action_min);
  TRACE("Action max: " << action_max);

  // Prepare actuator indexes for easy connection between actions of the target environment and agent actions
  bh_->setActuatorInterface(int_actuator, int_actuator_sym);
  bh_->setActuationMode(int_actuator.mode);
}

void LeoBaseEnvironment::fillObserve(const std::vector<CGenericStateVar> &genericStates,
                                      const std::vector<std::string> &observeList,
                                      std::vector<std::string> &observed) const
{
  std::vector<std::string>::const_iterator listMember = observeList.begin();
  std::vector<CGenericStateVar>::const_iterator gState;
  std::string::const_iterator it;
  for (; listMember < observeList.end(); listMember++)
  {
    bool found = false;
    gState = genericStates.begin();
    for (int i = 0; gState < genericStates.end(); gState++, i++)
    {
      const std::string &name = gState->name();
      it = std::search(name.begin(), name.end(), listMember->begin(), listMember->end());

      if (it != name.end())
      {
        it += listMember->size(); // point at the end of substring
        if (it == name.end() || *it == '.')
        {
          INFO("Adding to the observation vector (physical state): " << name);
          observed.push_back(name);
          found = true;
        }
      }
    }

    if (!found)
    {
      INFO("Adding to the observation vector (augmented state): " << *listMember);
      observed.push_back(*listMember);
    }
  }
}

void LeoBaseEnvironment::fillActuate(const std::vector<CGenericActionVar> &genericAction,
                                     const std::vector<std::string> &actuateList,
                                     TargetInterface::ActuatorInterface &int_actuator) const
{
  int_actuator.action.resize(genericAction.size());
  for (int i = 0; i < int_actuator.action.size(); i++) int_actuator.action[i] = -1;
  std::vector<std::string>::const_iterator listMember = actuateList.begin();
  std::vector<CGenericActionVar>::const_iterator gAction;
  std::string::const_iterator it;

  for (int j = 0; listMember < actuateList.end(); listMember++, j++)
  {
    // default case
    bool found = false;
    gAction = genericAction.begin();
    for (int i = 0; gAction < genericAction.end(); gAction++, i++)
    {
      std::string name = gAction->name();
      it = std::search(name.begin(), name.end(), listMember->begin(), listMember->end());

      if (it != name.end())
      {
        it += listMember->size(); // point at the end of substring
        INFO("Adding to the actuation vector: " << name);
        int_actuator.action[i] = j;
        found = true;
      }
    }

    // special case
    if (*listMember == "stanceknee")
    {
      // request update of both knees with the same action
      gAction = genericAction.begin();
      for (int i = 0; gAction < genericAction.end(); gAction++, i++)
      {
        std::string name = gAction->name();
        if (name.find("kneeleft") != std::string::npos)
        {
          INFO("Adding to the actuation vector: " << "kneeleft");
          int_actuator.action[i] = j;
        }
        if (name.find("kneeright") != std::string::npos)
        {
          INFO("Adding to the actuation vector: " << "kneeright");
          int_actuator.action[i] = j;
        }
      }

      // stance leg knee value will be overwritten with autoactuated control
      int_actuator.autoActuated.push_back("stanceknee");
      INFO("Adding auto-actuated joint '" << "stanceknee" << "'");

      found = true;
    }

    // exception case
    if (!found)
    {
      ERROR("Unknown actuaton joint '" << *listMember << "'");
      throw bad_param("leobase:listMember");
    }
  }

  // Adding auto-actuated joints
  for (int i = 0; i < int_actuator.action.size(); i++)
  {
    if (int_actuator.action[i] == -1)
    {
      std::string name = genericAction[i].name();
      std::replace( name.begin(), name.end(), '.', ' ');
      std::vector<std::string> cuttedName = cutLongStr(name);
      int_actuator.autoActuated.push_back(cuttedName[1]);
      INFO("Adding auto-actuated joint '" << cuttedName[1] << "'");
    }
  }

  // Assign actuation mode, should be common between all joints
  gAction = genericAction.begin();
  ESTGActuationMode actuationMode = gAction->mode();
  for (gAction++; gAction < genericAction.end(); gAction++)
    if (actuationMode != gAction->mode())
    {
      ERROR("Actuation mode is not coherent between joints, e.g. '" << gAction->name() << "'");
      throw bad_param("leobase:actuationMode");
    }
  int_actuator.mode = actuationMode;
  INFO("Actuation mode " << int_actuator.mode);
}
