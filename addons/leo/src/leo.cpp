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

/////////////////////////////////

LeoBaseEnvironment::LeoBaseEnvironment() :
  target_env_(NULL),
  observation_dims_(CLeoBhBase::svNumStates),
  time_test_(0),
  time_learn_(0),
  time0_(0),
  test_(0),
  exporter_(NULL)
{
}

void LeoBaseEnvironment::request(ConfigurationRequest *config)
{
  config->push_back(CRP("xml", "XML configuration filename", xml_));
  config->push_back(CRP("target_env", "environment", "Interaction environment", target_env_));
  config->push_back(CRP("observe", "string.observe", "Comma-separated list of state elements observed by an agent"));
  config->push_back(CRP("actuate", "string.actuate", "Comma-separated list of action elements provided by an agent"));
  config->push_back(CRP("exporter", "exporter", "Optional exporter for transition log (supports time, state, observation, action, reward, terminal)", exporter_, true));

  config->push_back(CRP("observation_dims", "int.observation_dims", "Number of observation dimensions", target_observation_dims_));
  config->push_back(CRP("action_dims", "int.action_dims", "Number of action dimensions", target_action_dims_));
}

void LeoBaseEnvironment::configure(Configuration &config)
{
  // Setup path to a configuration file
  //xml_ = std::string(LEO_CONFIG_DIR) + "/" + config["xml"].str();
  xml_ = config["xml"].str();
  std::cout << xml_ << std::endl;

  target_env_ = (Environment*)config["target_env"].ptr(); // here we can select an actual Leo enviromnent (simulation/real)

  target_observation_dims_ = config["observation_dims"];
  target_action_dims_ = config["action_dims"];

  exporter_ = (Exporter*) config["exporter"].ptr();
  if (exporter_)
    exporter_->init({"time", "state0", "state1", "action", "reward", "terminal"});

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

  // Creat ode object which resolves states and actions
  ode_ = new ODESTGEnvironment();
  if (!ode_->configure(config))
  {
    ERROR("Could not initialize STG ODE environment");
    return;
  }

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
  double &time  = test_?time_test_ :time_learn_;
  os << bh_->getProgressReport(time-time0_);
}

///////////////////////////////////////////
/// Helper functions
///
void LeoBaseEnvironment::config_parse_observations(Configuration &config)
{
  std::string observe = config["observe"].str();
  const std::vector<std::string> observeList = cutLongStr(observe);
  fillObserve(ode_->getSensors(), observeList, observe_);
  if (observe_.size() != target_observation_dims_)
    throw bad_param("leobase:observe");
  observation_dims_ = (observe_.array() != 0).count();

  // mask observation min/max vectors
  Vector ode_observation_min, ode_observation_max, observation_min, observation_max;
  config.get("observation_min", ode_observation_min);
  config.get("observation_max", ode_observation_max);
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
}

void LeoBaseEnvironment::config_parse_actions(Configuration &config)
{
  std::string actuate = config["actuate"].str();
  std::vector<std::string> actuateList = cutLongStr(actuate);
  fillActuate(ode_->getActuators(), actuateList, actuate_);
  if (actuate_.size() != target_action_dims_)
    throw bad_param("leobase:actuate");
  action_dims_ = (actuate_.array() != 0).count();

  // mask observation min/max vectors
  Vector target_action_min, target_action_max, action_min, action_max;
  config.get("action_min", target_action_min);
  config.get("action_max", target_action_max);
  action_min.resize(action_dims_);
  action_max.resize(action_dims_);
  for (int i = 0, j = 0; i < actuate_.size(); i++)
    if (actuate_[i])
    {
      action_min[j]   = target_action_min[i];
      action_max[j++] = target_action_max[i];
    }

  config.set("action_dims", action_dims_);
  config.set("action_min", action_min);
  config.set("action_max", action_max);
}

void LeoBaseEnvironment::fillObserve( const std::vector<CGenericStateVar> &genericStates,
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
      throw bad_param("leobase:observe");
    }
  }
}

void LeoBaseEnvironment::fillActuate(const std::vector<CGenericActionVar> &genericAction,
                                     const std::vector<std::string> &actuateList,
                                     Vector &out,
                                     const std::string *req,
                                     std::vector<int>  *reqIdx) const
{
  out.resize(genericAction.size());
  for (int i = 0; i < out.size(); i++) out[i] = 0;
  std::vector<std::string>::const_iterator listMember = actuateList.begin();
  std::vector<CGenericActionVar>::const_iterator gAction;
  std::string::const_iterator it;

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
      throw bad_param("leosim:actuate");
    }
  }
}
