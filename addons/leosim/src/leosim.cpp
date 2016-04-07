#include <XMLConfiguration.h>

#include <grl/environments/leosim/leosim.h>


using namespace grl;

REGISTER_CONFIGURABLE(LeoSimEnvironment)

void CGrlLeoBhWalkSym::resetState()
{
  mIsObserving          = false;
  mLastRewardedFoot     = lpFootLeft;
  mLastStancelegWasLeft = -1;
  mFootstepLength       = 0.0;
  mLastFootstepLength   = 0.0;

  // Reset velocity filters to zero velocity (this is the result of robot->setIC)
  for (int iJoint=0; iJoint<ljNumJoints; iJoint++)
    mJointSpeedFilter[iJoint].clear();

  for (int i=0; i<ljNumDynamixels; i++)
    mJointSpeedFilter[i].init(1.0/mTotalStepTime, 10.0);
  mJointSpeedFilter[ljTorso].init(mTotalStepTime, 25.0);	// 25Hz because? : 1) this encoder has 8x the resolution of a dynamixel 2) torso angles/velocities are more important
}

void CGrlLeoBhWalkSym::fillLeoState(const Vector &obs, const Vector &action, CLeoState &leoState)
{
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

  // required for correct energy calculation in reward function
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

void CGrlLeoBhWalkSym::parseLeoState(const CLeoState &leoState, Vector &obs)
{
  obs[siTorsoAngle]           = leoState.mJointAngles[ljTorso];
  obs[siTorsoAngleRate]       = leoState.mJointSpeeds[ljTorso];
  obs[siHipStanceAngle]       = leoState.mJointAngles[mHipStance];
  obs[siHipStanceAngleRate]   = leoState.mJointSpeeds[mHipStance];
  obs[siHipSwingAngle]        = leoState.mJointAngles[mHipSwing];
  obs[siHipSwingAngleRate]    = leoState.mJointSpeeds[mHipSwing];
  obs[siKneeStanceAngle]      = leoState.mJointAngles[mKneeStance];
  obs[siKneeStanceAngleRate]  = leoState.mJointSpeeds[mKneeStance];
  obs[siKneeSwingAngle]       = leoState.mJointAngles[mKneeSwing];
  obs[siKneeSwingAngleRate]   = leoState.mJointSpeeds[mKneeSwing];
}

void CGrlLeoBhWalkSym::setCurrentSTGState(CLeoState *leoState)
{
  mCurrentSTGState = leoState;
}

void CGrlLeoBhWalkSym::setPreviousSTGState(CLeoState *leoState)
{
  mPreviousSTGState = *leoState;
}

void CGrlLeoBhWalkSym::updateDerivedStateVars(CLeoState* currentSTGState)
{
  CLeoBhWalkSym::updateDerivedStateVars(currentSTGState);
}

/////////////////////////////////

LeoSimEnvironment::LeoSimEnvironment() :
  bhWalk_(&leoSim_),
  observation_dims_(CGrlLeoBhWalkSym::svNumStates),
  requested_action_dims_(CGrlLeoBhWalkSym::svNumActions),
  learn_stance_knee_(0),
  time_test_(0),
  time_learn_(0),
  test_(0),
  exporter_(NULL)
{
}

void LeoSimEnvironment::request(ConfigurationRequest *config)
{
  ODEEnvironment::request(config);

  config->push_back(CRP("observe", "string.observe_", "Comma-separated list of state elements observed by an agent"));
  config->push_back(CRP("actuate", "string.actuate_", "Comma-separated list of action elements provided by an agent"));
  config->push_back(CRP("learn_stance_knee", "Learn stance knee", learn_stance_knee_, CRP::Configuration, 0, 1));
  config->push_back(CRP("exporter", "exporter", "Optional exporter for transition log (supports time, state, observation, action, reward, terminal)", exporter_, true));
}

void LeoSimEnvironment::configure(Configuration &config)
{
  // Setup path to a configuration file
  std::string xml = std::string(LEOSIM_CONFIG_DIR) + "/" + config["xml"].str();
  //config["xml"].str() = xml; // update for the correct ODESIM initialization
  config.set("xml", xml);

  // Read yaml first. Settings will be overwritten by ODEEnvironment::configure,
  // which are different because they belong to ODE simulator.
  ODEEnvironment::configure(config);
  ode_observation_dims_ = config["observation_dims"];
  ode_action_dims_ = config["action_dims"];
  learn_stance_knee_ = config["learn_stance_knee"];

  exporter_ = (Exporter*) config["exporter"].ptr();
  if (exporter_)
    exporter_->init({"time", "state0", "state1", "action", "reward", "terminal"});

  // Process configuration for leosim
  CXMLConfiguration xmlConfig;
  if (!xmlConfig.loadFile(xml))
  {
    ERROR("Couldn't load XML configuration file \"" << xml << "\"!\nPlease check that the file exists and that it is sound (error: " << xmlConfig.errorStr() << ").");
    return;
  }

  // Resolve expressions
  xmlConfig.resolveExpressions();
  
  // Read rewards and preprogrammed angles
  bhWalk_.readConfig(xmlConfig.root());

  // Parse observations
  std::string observe = config["observe"].str();
  const std::vector<std::string> observeList = cutLongStr(observe);
  fillObserve(env_->getSensors(), observeList, observe_);
  if (observe_.size() != ode_observation_dims_)
    throw bad_param("leosim/walk:observe");
  observation_dims_ = (observe_.array() != 0).count();

  // mask observation min/max vectors
  Vector ode_observation_min = config["observation_min"], observation_min;
  Vector ode_observation_max = config["observation_max"], observation_max;
  observation_min.resize(observation_dims_);
  observation_max.resize(observation_dims_);
  for (int i = 0, j = 0; i < observe_.size(); i++)
    if (observe_[i])
    {
      observation_min[j]   = ode_observation_min[i];
      observation_max[j++] = ode_observation_max[i];
    }
  config.set("observation_dims", observation_dims_);
  config.set("observation_min", observation_min);
  config.set("observation_max", observation_max);

  // Parse actions
  std::vector<int> knee_idx;
  int omit_knee_idx = -1;
  std::string actuate = config["actuate"].str();
  std::vector<std::string> actuateList = cutLongStr(actuate);
  fillActuate(env_->getActuators(), actuateList, actuate_, knee_idx);
  if (actuate_.size() != ode_action_dims_)
    throw bad_param("leosim/walk:actuate_");
  requested_action_dims_ = (actuate_.array() != 0).count();
  if (learn_stance_knee_)
    action_dims_ = requested_action_dims_;
  else
  {
    if (knee_idx.size() != 2)
      throw bad_param("leosim/walk:actuate_ (if any of knees is learnt, then always include both knees)");
    action_dims_ = requested_action_dims_ - 1;
    omit_knee_idx = knee_idx[1];
  }

  // mask observation min/max vectors
  Vector ode_action_min = config["action_min"], action_min;
  Vector ode_action_max = config["action_max"], action_max;
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
  ode_obs_.resize(ode_observation_dims_);
  ode_action_.resize(ode_action_dims_);
}

void LeoSimEnvironment::reconfigure(const Configuration &config)
{
  ODEEnvironment::reconfigure(config);
  time_test_ = time_learn_ = 0;
}

LeoSimEnvironment *LeoSimEnvironment::clone()
{
  return new LeoSimEnvironment(*this);
}

void LeoSimEnvironment::start(int test, Vector *obs)
{
  test_ = test;

  ODEEnvironment::start(test, &ode_obs_);

  bhWalk_.resetState();

  // Parse obs into CLeoState (Start with left leg being the stance leg)
  bhWalk_.fillLeoState(ode_obs_, Vector(), leoState_);
  bhWalk_.setCurrentSTGState(&leoState_);
  bhWalk_.setPreviousSTGState(&leoState_);

  // update derived state variables
  bhWalk_.updateDerivedStateVars(&leoState_); // swing-stance switching happens here

  // construct new obs from CLeoState
  obs->resize(observation_dims_);
  bhWalk_.parseLeoState(leoState_, *obs);

  bhWalk_.setCurrentSTGState(NULL);

  if (exporter_)
    exporter_->open((test_?"test":"learn"), (test_?time_test_:time_learn_) != 0.0);
}

double LeoSimEnvironment::step(const Vector &action, Vector *obs, double *reward, int *terminal)
{
  double &time = test_?time_test_:time_learn_;
  bhWalk_.setCurrentSTGState(&leoState_);

  // auto actuate unlearned joints to find complete action vector
  double actionArm, actionStanceKnee, actionSwingKnee, actionStanceHip, actionSwingHip;
  actionStanceHip = action[0];
  actionSwingHip  = action[1];
  if (!learn_stance_knee_)
  {
    // Auto actuation of the stance knee
    actionStanceKnee = bhWalk_.grlAutoActuateKnee();
    actionSwingKnee  = action[2];
  }
  else
  {
    // Learn both actions
    actionStanceKnee = action[2];
    actionSwingKnee  = action[3];
  }
  Vector actionAnkles;
  bhWalk_.grlAutoActuateAnkles(actionAnkles);
  actionArm = bhWalk_.grlAutoActuateArm();

  // concatenation happens in the order of <actionvar> definitions in an xml file
  // shoulder, right hip, left hip, right knee, left knee, right ankle, left ankle
  if (bhWalk_.stanceLegLeft())
    ode_action_ << actionArm, actionSwingHip, actionStanceHip, actionSwingKnee, actionStanceKnee, actionAnkles;
  else
    ode_action_ << actionArm, actionStanceHip, actionSwingHip, actionStanceKnee, actionSwingKnee, actionAnkles;

  bhWalk_.setPreviousSTGState(&leoState_);
  TRACE("ode action = " << ode_action_);
  double tau = ODEEnvironment::step(ode_action_, &ode_obs_, reward, terminal);
  TRACE("ode observation = " << ode_obs_);

  // Filter joint speeds
  // Parse obs into CLeoState
  bhWalk_.fillLeoState(ode_obs_, ode_action_, leoState_);
  bhWalk_.setCurrentSTGState(&leoState_);

  // update derived state variables
  bhWalk_.updateDerivedStateVars(&leoState_);

  // construct new obs from CLeoState
  bhWalk_.parseLeoState(leoState_, *obs);

  // Determine reward
  *reward = bhWalk_.calculateReward();

  // Debug info
  std::vector<double> s1(leoState_.mJointAngles, leoState_.mJointAngles + ljNumJoints);
  std::vector<double> v1(leoState_.mJointSpeeds, leoState_.mJointSpeeds + ljNumJoints);
  std::vector<double> a(leoState_.mActuationVoltages, leoState_.mActuationVoltages + ljNumDynamixels);
/*
  std::cout << "State angles: " << s1 << std::endl;
  std::cout << "State velocities: " << v1 << std::endl;
  std::cout << "Contacts: " << (int)leoState_.mFootContacts << std::endl;

  std::cout << "RL action: " << action << std::endl;
  std::cout << "Full action: " << a << std::endl;

  std::cout << "Reward: " << *reward << std::endl;
*/
  // ... and termination
  if (*terminal == 1) // timeout
    *terminal = 1;
  else if (bhWalk_.isDoomedToFall(&leoState_, false))
    *terminal = 2;
  else
    *terminal = 0;

  // export
  std::vector<double> s0(bhWalk_.getPreviousSTGState()->mJointAngles, bhWalk_.getPreviousSTGState()->mJointAngles + ljNumJoints);
  std::vector<double> v0(bhWalk_.getPreviousSTGState()->mJointSpeeds, bhWalk_.getPreviousSTGState()->mJointSpeeds + ljNumJoints);
  s0.insert(s0.end(), v0.begin(), v0.end());
  s1.insert(s1.end(), v1.begin(), v1.end());
  if (exporter_)
    exporter_->write({grl::VectorConstructor(time), grl::VectorConstructor(s0),  grl::VectorConstructor(s1),
                      grl::VectorConstructor(a), grl::VectorConstructor(*reward), grl::VectorConstructor(*terminal)
                     });

  time += tau;

  return tau;
}

void LeoSimEnvironment::fillObserve( const std::vector<CGenericStateVar> &genericStates,
                                     const std::vector<std::string> &observeList,
                                     Vector &out) const
{
  out.resize(genericStates.size());
  for (int i = 0; i < out.size(); i++) out[i] = 0;
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
          INFO("Adding to the observation vector: " << name);
          out[i] = 1;
          found = true;
        }
      }
    }

    if (!found)
    {
      ERROR("Requested unregistered field '" << *listMember << "'");
      throw bad_param("leosim:observe");
    }
  }
}

void LeoSimEnvironment::fillActuate(const std::vector<CGenericActionVar> &genericAction,
                                     const std::vector<std::string> &actuateList,
                                     Vector &out, std::vector<int> &knee_idx) const
{
  out.resize(genericAction.size());
  for (int i = 0; i < out.size(); i++) out[i] = 0;
  std::vector<std::string>::const_iterator listMember = actuateList.begin();
  std::vector<CGenericActionVar>::const_iterator gAction;
  std::string::const_iterator it;
  std::string knee_str = "knee";

  for (; listMember < actuateList.end(); listMember++)
  {
    bool found = false;
    gAction = genericAction.begin();
    for (int i = 0; gAction < genericAction.end(); gAction++, i++)
    {
      const std::string &name = gAction->name();
      it = std::search(name.begin(), name.end(), listMember->begin(), listMember->end());

      if (it != name.end())
      {
        it += listMember->size(); // point at the end of substring
        if (it == name.end() || *it == '.')
        {
          INFO("Adding to the actuation vector: " << name);
          out[i] = 1;
          if (std::search(listMember->begin(), listMember->end(), knee_str.begin(), knee_str.end()) != listMember->end())
            knee_idx.push_back(i);
          found = true;
        }
      }
    }

    if (!found)
    {
      ERROR("Requested unregistered field '" << *listMember << "'");
      throw bad_param("leosim:actuate");
    }
  }
}
