#include <sys/stat.h>
#include <XMLConfiguration.h>
#include <grl/environments/leo/leo.h>

using namespace grl;

void CLeoBhBase::resetState()
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

  leoState.mFootContacts  = obs[svRightToeContact]?LEO_FOOTSENSOR_RIGHT_TOE:0;
  leoState.mFootContacts |= obs[svRightHeelContact]?LEO_FOOTSENSOR_RIGHT_HEEL:0;
  leoState.mFootContacts |= obs[svLeftToeContact]?LEO_FOOTSENSOR_LEFT_TOE:0;
  leoState.mFootContacts |= obs[svLeftHeelContact]?LEO_FOOTSENSOR_LEFT_HEEL:0;

  // required for the correct energy calculation in the reward function
  if (action.size())
  {
    leoState.mActuationVoltages[ljShoulder]   = action[avLeftArmTorque];
    leoState.mActuationVoltages[ljHipRight]   = action[avRightHipTorque];
    leoState.mActuationVoltages[ljHipLeft]    = action[avLeftHipTorque];
    leoState.mActuationVoltages[ljKneeRight]  = action[avRightKneeTorque];
    leoState.mActuationVoltages[ljKneeLeft]   = action[avLeftKneeTorque];
    leoState.mActuationVoltages[ljAnkleRight] = action[avRightAnkleTorque];
    leoState.mActuationVoltages[ljAnkleLeft]  = action[avLeftAnkleTorque];
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
  observation_dims_(CLeoBhBase::svNumStates),
  time_test_(0),
  time_learn_(0),
  time0_(0),
  test_(0),
  exporter_(NULL),
  bh_(NULL)
{
}

void LeoBaseEnvironment::request(ConfigurationRequest *config)
{
  config->push_back(CRP("xml", "XML configuration filename", xml_));
  config->push_back(CRP("target_env", "environment", "Interaction environment", target_env_));
  config->push_back(CRP("observe", "string.observe", "Comma-separated list of state elements observed by an agent"));
  config->push_back(CRP("actuate", "string.actuate", "Comma-separated list of action elements provided by an agent"));
  config->push_back(CRP("exporter", "exporter", "Optional exporter for transition log (supports time, state, observation, action, reward, terminal)", exporter_, true));

  // TODO: Can we get these automatically from a target environment?
  config->push_back(CRP("observation_dims", "int.observation_dims", "Number of observation dimensions", target_observation_dims_));
  config->push_back(CRP("action_dims", "int.action_dims", "Number of action dimensions", target_action_dims_));
  config->push_back(CRP("observation_min", "vector.observation_min", "Lower limit of observations", observation_min_, CRP::System));
  config->push_back(CRP("observation_max", "vector.observation_max", "Upper limit of observations", observation_max_, CRP::System));
  config->push_back(CRP("action_min", "vector.action_min", "Lower limit of action", action_min_, CRP::System));
  config->push_back(CRP("action_max", "vector.action_max", "Upper limit of action", action_max_, CRP::System));
}

void LeoBaseEnvironment::configure(Configuration &config)
{
  target_env_ = (Environment*)config["target_env"].ptr(); // here we can select an actual Leo enviromnent (simulation/real)

  target_observation_dims_ = config["observation_dims"];
  target_action_dims_ = config["action_dims"];

  exporter_ = (Exporter*) config["exporter"].ptr();
  if (exporter_)
    exporter_->init({"time", "state0", "state1", "action", "reward", "terminal"});

  /////////////////////////////////////////////////
  // Setup path to a configuration file
  xml_ = config["xml"].str();
  struct stat buffer;
  if (stat (xml_.c_str(), &buffer) != 0)
    xml_ = std::string(LEO_CONFIG_DIR) + "/" + config["xml"].str();
  std::cout << xml_ << std::endl;

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

  std::vector<std::string> sensors =    { std::string("robot.torso_boom.angle"), std::string("robot.torso_boom.anglerate"),
                                          std::string("robot.shoulder.angle"),   std::string("robot.shoulder.anglerate"),
                                          std::string("robot.hipright.angle"),   std::string("robot.hipright.anglerate"),
                                          std::string("robot.hipleft.angle"),    std::string("robot.hipleft.anglerate"),
                                          std::string("robot.kneeright.angle"),  std::string("robot.kneeright.anglerate"),
                                          std::string("robot.kneeleft.angle"),   std::string("robot.kneeleft.anglerate"),
                                          std::string("robot.ankleright.angle"), std::string("robot.ankleright.anglerate"),
                                          std::string("robot.ankleleft.angle"),  std::string("robot.ankleleft.anglerate"),
                                          std::string("robot.toeright.contact"), std::string("robot.heelright.contact"),
                                          std::string("robot.toeleft.contact"),  std::string("robot.heelleft.contact")
                                        };

  std::vector<std::string> actuators =  { std::string("robot.shoulder.voltage"),
                                          std::string("robot.hipright.voltage"),  std::string("robot.hipleft.voltage"),
                                          std::string("robot.kneeright.voltage"), std::string("robot.kneeleft.voltage"),
                                          std::string("robot.ankleright.voltage"),std::string("robot.ankleleft.voltage")
                                        };

  // Select states and actions that are delivered to an agent
  configParseObservations(config, sensors);
  configParseActions(config, actuators);

  // reserve memory
  target_obs_.resize(target_observation_dims_);
  target_action_.resize(target_action_dims_);
}

void LeoBaseEnvironment::reconfigure(const Configuration &config)
{
  time_test_ = time_learn_ = time0_ = 0;
}

LeoBaseEnvironment *LeoBaseEnvironment::clone() const
{
  return NULL;
}

void LeoBaseEnvironment::start(int test)
{
  test_ = test;
  bh_->resetState();

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
  std::vector<double> a(leoState_.mActuationVoltages, leoState_.mActuationVoltages + ljNumDynamixels);

  if (exporter_)
  {
    std::vector<double> s0(bh_->getPreviousSTGState()->mJointAngles, bh_->getPreviousSTGState()->mJointAngles + ljNumJoints);
    std::vector<double> v0(bh_->getPreviousSTGState()->mJointSpeeds, bh_->getPreviousSTGState()->mJointSpeeds + ljNumJoints);
    s0.insert(s0.end(), v0.begin(), v0.end());
    s1.insert(s1.end(), v1.begin(), v1.end());

    Vector s0v, s1v, av;
    toVector(s0, s0v);
    toVector(s1, s1v);
    toVector(a, av);

    exporter_->write({grl::VectorConstructor(time), s0v,  s1v,
                      av, grl::VectorConstructor(reward), grl::VectorConstructor(terminal)
                     });
  }

  TRACE("State angles: " << s1);
  TRACE("State velocities: " << v1);
  TRACE("Contacts: " << (int)leoState_.mFootContacts);
  TRACE("Full action: " << a);
  TRACE("Reward: " << reward);

  time += tau;
}

void LeoBaseEnvironment::report(std::ostream &os)
{
  double trialTime  = test_?time_test_:time_learn_ - time0_;
  os << bh_->getProgressReport(trialTime);
}

///////////////////////////////////////////
/// Helper functions
///
void LeoBaseEnvironment::configParseObservations(Configuration &config, const std::vector<std::string> &sensors)
{
  const std::vector<std::string> observeList = cutLongStr( config["observe"].str() );
  std::vector<std::string> observe;
  ObserverStruct observer_struct;
  fillObserve(sensors, observeList, observe);
  fillObserverStruct(observe, observer_struct);
  observation_dims_ = observe.size();

  // mask observation min/max vectors
  Vector target_observation_min, target_observation_max, observation_min, observation_max;
  config.get("observation_min", target_observation_min);
  config.get("observation_max", target_observation_max);
  observation_min.resize(observation_dims_);
  observation_max.resize(observation_dims_);

  int i, j;
  for (i = 0; i < observer_struct.angles.size(); i++)
  {
    std::string name = "robot." + bh_->jointIndexToName(observer_struct.angles[i]) + ".angle";
    int sensor_idx = findVarIdx(sensors, name);
    observation_min[i] = target_observation_min[sensor_idx];
    observation_max[i] = target_observation_max[sensor_idx];
  }
  for (j = 0; j < observer_struct.angle_rates.size(); j++)
  {
    std::string name = "robot." + bh_->jointIndexToName(observer_struct.angle_rates[j]) + ".anglerate";
    int sensor_idx = findVarIdx(sensors, name);
    observation_min[i+j] = target_observation_min[sensor_idx];
    observation_max[i+j] = target_observation_max[sensor_idx];
  }

  // Set parameters exported to an agent
  config.set("observation_dims", observation_dims_);
  config.set("observation_min", observation_min);
  config.set("observation_max", observation_max);

  INFO("Environment observation dims: " << observation_dims_);
  INFO("Environment observation min: "  << observation_min);
  INFO("Environment observation max: "  << observation_max);

  // Prepare observer indexes for easy connection between states of the target environment and agent observations
  bh_->setObserverStruct(observer_struct);
}

int LeoBaseEnvironment::findVarIdx(const std::vector<std::string> &genericStates, std::string query) const
{
  std::vector<std::string>::const_iterator gState = genericStates.begin();
  for (int i = 0; gState < genericStates.end(); gState++, i++)
    if (query == *gState)
      return i;
  return -1;
}

void LeoBaseEnvironment::fillObserverStruct(const std::vector<std::string> &observer_names, ObserverStruct &observer_idx) const
{
  for (int i = 0; i < observer_names.size(); i++)
  {
    std::string name = observer_names[i];
    std::replace( name.begin(), name.end(), '.', ' ');
    std::vector<std::string> cuttedName = cutLongStr(name);
    if (cuttedName.size() == 1)
      observer_idx.augmented.push_back(name);
    else if (cuttedName[2] == "angle")
      observer_idx.angles.push_back(bh_->jointNameToIndex(cuttedName[1]));
    else if (cuttedName[2] == "anglerate")
      observer_idx.angle_rates.push_back(bh_->jointNameToIndex(cuttedName[1]));
    else
    {
      ERROR("Unknown joint '" << cuttedName[2] << "'");
      throw bad_param("leobase:cuttedName[2]");
    }
  }
}

void LeoBaseEnvironment::configParseActions(Configuration &config, const std::vector<std::string> &actuators)
{
  Vector actuate;
  std::vector<std::string> actuateList = cutLongStr(config["actuate"].str());
  fillActuate(actuators, actuateList, actuate);
  if (actuate.size() != target_action_dims_)
    throw bad_param("leobase:actuate");
  action_dims_ = (actuate.array() != 0).count();

  // mask observation min/max vectors
  Vector target_action_min, target_action_max, action_min, action_max;
  config.get("action_min", target_action_min);
  config.get("action_max", target_action_max);
  action_min.resize(action_dims_);
  action_max.resize(action_dims_);
  for (int i = 0, j = 0; i < actuate.size(); i++)
    if (actuate[i])
    {
      action_min[j]   = target_action_min[i];
      action_max[j++] = target_action_max[i];
    }

  // Set parameters exported to an agent
  config.set("action_dims", action_dims_);
  config.set("action_min", action_min);
  config.set("action_max", action_max);

  INFO("Environment action dims: " << action_dims_);
  INFO("Environment action min: "  << action_min);
  INFO("Environment action max: "  << action_max);
}

void LeoBaseEnvironment::fillObserve(const std::vector<std::string> &genericStates,
                                      const std::vector<std::string> &observeList,
                                      std::vector<std::string> &observe) const
{
  std::vector<std::string>::const_iterator listMember = observeList.begin();
  std::vector<std::string>::const_iterator gState;
  std::string::const_iterator it;
  for (; listMember < observeList.end(); listMember++)
  {
    bool found = false;
    gState = genericStates.begin();
    for (int i = 0; gState < genericStates.end(); gState++, i++)
    {
      const std::string &name = *gState;
      it = std::search(name.begin(), name.end(), listMember->begin(), listMember->end());

      if (it != name.end())
      {
        it += listMember->size(); // point at the end of substring
        if (it == name.end() || *it == '.')
        {
          INFO("Adding to the observation vector (physical state): " << name);
          observe.push_back(name);
          found = true;
        }
      }
    }

    if (!found)
    {
      INFO("Adding to the observation vector (augmented state): " << *listMember);
      observe.push_back(*listMember);
    }
  }
}

void LeoBaseEnvironment::fillActuate(const std::vector<std::string> &genericAction,
                                     const std::vector<std::string> &actuateList,
                                     Vector &out,
                                     const std::string *req,
                                     std::vector<int>  *reqIdx) const
{
  out.resize(genericAction.size());
  for (int i = 0; i < out.size(); i++) out[i] = 0;
  std::vector<std::string>::const_iterator listMember = actuateList.begin();
  std::vector<std::string>::const_iterator gAction;
  std::string::const_iterator it;

  for (; listMember < actuateList.end(); listMember++)
  {
    bool found = false;
    gAction = genericAction.begin();
    for (int i = 0; gAction < genericAction.end(); gAction++, i++)
    {
      const std::string &name = *gAction;
      it = std::search(name.begin(), name.end(), listMember->begin(), listMember->end());

      if (it != name.end())
      {
        it += listMember->size(); // point at the end of substring
        if (it == name.end() || *it == '.')
        {
          INFO("Adding to the actuation vector: " << name);
          out[i] = 1;
          if (req != NULL && reqIdx != NULL)
            if (std::search(listMember->begin(), listMember->end(), req->begin(), req->end()) != listMember->end())
              reqIdx->push_back(i);
          found = true;
        }
      }
    }

    if (!found)
    {
      ERROR("Requested unregistered field '" << *listMember << "'");
      throw bad_param("leobase:listMember");
    }
  }
}
