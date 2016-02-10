#include <XMLConfiguration.h>

#include <grl/environments/leosim/leosim.h>


using namespace grl;

REGISTER_CONFIGURABLE(LeoSimEnvironment)

void CGrlLeoBhWalkSym::init()
{
  // Init speed filters
  for (int i=0; i<ljNumDynamixels; i++)
    mJointSpeedFilter[i].init(1.0/mTotalStepTime, 10.0);
  mJointSpeedFilter[ljTorso].init(mTotalStepTime, 25.0);	// 25Hz because? : 1) this encoder has 8x the resolution of a dynamixel 2) torso angles/velocities are more important
}

void CGrlLeoBhWalkSym::resetState()
{
  mLastRewardedFoot     = lpFootLeft;
  mLastStancelegWasLeft = -1;
  mFootstepLength       = 0.0;
  mLastFootstepLength   = 0.0;

  // Reset velocity filters to zero velocity (this is the result of robot->setIC)
  for (int iJoint=0; iJoint<ljNumJoints; iJoint++)
    mJointSpeedFilter[iJoint].clear();
}

void CGrlLeoBhWalkSym::parseOdeObs(const Vector &obs, CLeoState &leoState)
{
  leoState.mJointAngles[ljTorso]      = obs[svTorsoAngle];
  leoState.mJointSpeeds[ljTorso]      = mJointSpeedFilter[ljTorso].filter(obs[svTorsoAngleRate]);
  leoState.mJointAngles[ljShoulder]   = obs[svShoulderAngle];
  leoState.mJointSpeeds[ljShoulder]   = mJointSpeedFilter[ljTorso].filter(obs[svShoulderAngleRate]);
  leoState.mJointAngles[ljHipRight]   = obs[svRightHipAngle];
  leoState.mJointSpeeds[ljHipRight]   = mJointSpeedFilter[ljHipLeft].filter(obs[svRightHipAngleRate]);
  leoState.mJointAngles[ljHipLeft]    = obs[svLeftHipAngle];
  leoState.mJointSpeeds[ljHipLeft]    = mJointSpeedFilter[ljHipRight].filter(obs[svLeftHipAngleRate]);
  leoState.mJointAngles[ljKneeRight]  = obs[svRightKneeAngle];
  leoState.mJointSpeeds[ljKneeRight]	= mJointSpeedFilter[ljKneeRight].filter(obs[svRightKneeAngleRate]);
  leoState.mJointAngles[ljKneeLeft]   = obs[svLeftKneeAngle];
  leoState.mJointSpeeds[ljKneeLeft]   = mJointSpeedFilter[ljKneeLeft].filter(obs[svLeftKneeAngleRate]);

  if (mAnkleStance == ljAnkleLeft)
  {
    // Left leg is a stance one
    leoState.mFootContacts = LEO_FOOTSENSOR_LEFT_HEEL | LEO_FOOTSENSOR_LEFT_TOE;
  }
  else
  {
    // Right leg is a stance one
    leoState.mFootContacts = LEO_FOOTSENSOR_RIGHT_HEEL | LEO_FOOTSENSOR_RIGHT_TOE;
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
  bhWalk_(new CSTGLeoSim()),
  observation_dims_(CGrlLeoBhWalkSym::svNumStates),
  action_dims_(CGrlLeoBhWalkSym::svNumActions)
{

}

void LeoSimEnvironment::request(ConfigurationRequest *config)
{
  ODEEnvironment::request(config);

  config->push_back(CRP("observe", "string.observe_", "Comma-separated list of state elements observed by an agent"));
  config->push_back(CRP("actuate", "string.actuate_", "Comma-separated list of action elements provided by an agent"));
}

void LeoSimEnvironment::fillObserve( const std::vector<CGenericStateVar> &genericStates,
                                     const std::vector<std::string> &observeList,
                                     Vector &out)
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
      it = search(name.begin(), name.end(), listMember->begin(), listMember->end());

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

void LeoSimEnvironment::fillActuate( const std::vector<CGenericActionVar> &genericAction,
                                     const std::vector<std::string> &actuateList,
                                     Vector &out)
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
      CGenericActionVar a = *gAction;
      //a.mValue
      const std::string &name = gAction->name();
      it = search(name.begin(), name.end(), listMember->begin(), listMember->end());

      if (it != name.end())
      {
        it += listMember->size(); // point at the end of substring
        if (it == name.end() || *it == '.')
        {
          INFO("Adding to the actuation vector: " << name);
          out[i] = 1;
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

void LeoSimEnvironment::configure(Configuration &config)
{
  // Read yaml first. Settings will be overwritten by ODEEnvironment::configure,
  // which are different because they belong to ODE simulator.

  ODEEnvironment::configure(config);
  ode_observation_dims_ = config["observation_dims"];
  ode_action_dims_ = config["action_dims"];

  std::string xml = config["xml"].str();

  CXMLConfiguration xmlConfig;

  if (!xmlConfig.loadFile(xml))
  {
    ERROR("Couldn't load XML configuration file \"" << xml << "\"!\nPlease check that the file exists and that it is sound (error: " << xmlConfig.errorStr() << ").");
    return;
  }

  // Resolve expressions
  xmlConfig.resolveExpressions();
  
  // TODO: Read rewards and preprogrammed angles (LeoBhWalkSym.cpp:102)
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
    if (observe_[i] != 0)
    {
      observation_min[j]   = ode_observation_min[i];
      observation_max[j++] = ode_observation_max[i];
    }
  config.set("observation_dims", observation_dims_);
  config.set("observation_min", observation_min);
  config.set("observation_max", observation_max);

  // Parse actions
  std::string actuate = config["actuate"].str();
  std::vector<std::string> actuateList = cutLongStr(actuate);
  fillActuate(env_->getActuators(), actuateList, actuate_);
  if (actuate_.size() != ode_action_dims_)
    throw bad_param("leosim/walk:actuate");
  action_dims_ = (actuate_.array() != 0).count();

  // mask observation min/max vectors
  Vector ode_action_min = config["action_min"], action_min;
  Vector ode_action_max = config["action_max"], action_max;
  action_min.resize(action_dims_);
  action_max.resize(action_dims_);
  for (int i = 0, j = 0; i < actuate_.size(); i++)
    if (actuate_[i] != 0)
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
}

LeoSimEnvironment *LeoSimEnvironment::clone()
{
  return new LeoSimEnvironment(*this);
}

void LeoSimEnvironment::start(int test, Vector *obs)
{
  ODEEnvironment::start(test, &ode_obs_);

  bhWalk_.resetState();

  // TODO: Parse obs into CLeoState (Start with left leg being the stance leg)
  bhWalk_.parseOdeObs(ode_obs_, leoState_);
  bhWalk_.setCurrentSTGState(&leoState_);
  bhWalk_.setPreviousSTGState(&leoState_);

  // TODO: update derived state variables (LeoBhWalkSym.cpp:281)
  bhWalk_.updateDerivedStateVars(&leoState_);

  // TODO: construct new obs from CLeoState (policy.cpp:162)
  obs->resize(observation_dims_);
  bhWalk_.parseLeoState(leoState_, *obs);

  bhWalk_.setCurrentSTGState(NULL);
}

double LeoSimEnvironment::step(const Vector &action, Vector *obs, double *reward, int *terminal)
{
  bhWalk_.setCurrentSTGState(&leoState_);

  // TODO: auto actuate unlearned joints to find complete action vector (LeoBhWalkSym.cpp:880)
  Vector autoActionShoulder, autoActionAnkles;
  bhWalk_.grlAutoActuateAnkles(autoActionAnkles);
  bhWalk_.grlAutoActuateArm(autoActionShoulder);

  // concatenation happens in the order of <actionvar> definitions in an xml file
  ode_action_ << autoActionShoulder, action, autoActionAnkles;

  TRACE("ode action = " << ode_action_);
  ODEEnvironment::step(ode_action_, &ode_obs_, reward, terminal);
  TRACE("ode observation = " << ode_obs_);

  // TODO: Filter joint speeds (STGLeoSim.cpp:275)
  // TODO: Parse obs into CLeoState
  bhWalk_.parseOdeObs(ode_obs_, leoState_);

  // TODO: update derived state variables (LeoBhWalkSym.cpp:281)
  bhWalk_.updateDerivedStateVars(&leoState_);

  // TODO: construct new obs from CLeoState (policy.cpp:162)
  bhWalk_.parseLeoState(leoState_, *obs);

  // TODO: Determine reward and termination (LeoBhWalkSym.cpp:693)
  *reward = bhWalk_.calculateReward();

  if (*terminal == 1) // timeout
    *terminal = 1;
  else if (bhWalk_.isDoomedToFall(&leoState_, false))
    *terminal = 2;
  else
    *terminal = 0;

  bhWalk_.setCurrentSTGState(NULL);
  bhWalk_.setPreviousSTGState(&leoState_);
}

