#include <math.h>

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

void CGrlLeoBhWalkSym::fillLeoState(const Vector &obs, const Vector &action, CLeoState *leoState)
{
  leoState->mJointAngles[ljTorso]      = -obs[svTorsoAngle];
  leoState->mJointSpeeds[ljTorso]      = -mJointSpeedFilter[ljTorso].filter(obs[svTorsoAngleRate]);
  leoState->mJointAngles[ljShoulder]   = obs[svLeftArmAngle];
  leoState->mJointSpeeds[ljShoulder]   = mJointSpeedFilter[ljShoulder].filter(obs[svLeftArmAngleRate]);
  leoState->mJointAngles[ljHipRight]   = obs[svRightHipAngle];
  leoState->mJointSpeeds[ljHipRight]   = mJointSpeedFilter[ljHipRight].filter(obs[svRightHipAngleRate]);
  leoState->mJointAngles[ljHipLeft]    = obs[svLeftHipAngle];
  leoState->mJointSpeeds[ljHipLeft]    = mJointSpeedFilter[ljHipLeft].filter(obs[svLeftHipAngleRate]);
  leoState->mJointAngles[ljKneeRight]  = obs[svRightKneeAngle];
  leoState->mJointSpeeds[ljKneeRight]	= mJointSpeedFilter[ljKneeRight].filter(obs[svRightKneeAngleRate]);
  leoState->mJointAngles[ljKneeLeft]   = obs[svLeftKneeAngle];
  leoState->mJointSpeeds[ljKneeLeft]   = mJointSpeedFilter[ljKneeLeft].filter(obs[svLeftKneeAngleRate]);
  leoState->mJointAngles[ljAnkleRight] = obs[svRightAnkleAngle];
  leoState->mJointSpeeds[ljAnkleRight] = mJointSpeedFilter[ljAnkleRight].filter(obs[svRightAnkleAngleRate]);
  leoState->mJointAngles[ljAnkleLeft]  = obs[svLeftAnkleAngle];
  leoState->mJointSpeeds[ljAnkleLeft]	= mJointSpeedFilter[ljAnkleLeft].filter(obs[svLeftAnkleAngleRate]);

  leoState->mFootContacts  = obs[svRightToeContact]?LEO_FOOTSENSOR_RIGHT_TOE:0;
  leoState->mFootContacts |= obs[svRightHeelContact]?LEO_FOOTSENSOR_RIGHT_HEEL:0;
  leoState->mFootContacts |= obs[svLeftToeContact]?LEO_FOOTSENSOR_LEFT_TOE:0;
  leoState->mFootContacts |= obs[svLeftHeelContact]?LEO_FOOTSENSOR_LEFT_HEEL:0;

  // required for correct energy calculation in reward function
  if (action.size())
  {
    leoState->mActuationVoltages[ljShoulder]   = action[avLeftArmTorque];
    leoState->mActuationVoltages[ljHipRight]   = action[avRightHipTorque];
    leoState->mActuationVoltages[ljHipLeft]    = action[avLeftHipTorque];
    leoState->mActuationVoltages[ljKneeRight]  = action[avRightKneeTorque];
    leoState->mActuationVoltages[ljKneeLeft]   = action[avLeftKneeTorque];
    leoState->mActuationVoltages[ljAnkleRight] = action[avRightAnkleTorque];
    leoState->mActuationVoltages[ljAnkleLeft]  = action[avLeftAnkleTorque];
  }
}

void CGrlLeoBhWalkSym::parseLeoState(const CLeoState &leoState, Vector *obs)
{
  *obs = ConstantVector(svNumStates, 0);
  
  // Basic values
  (*obs)[svTorsoAngle] = leoState.mJointAngles[ljTorso];
  (*obs)[svTorsoAngleRate] = leoState.mJointSpeeds[ljTorso];
  (*obs)[svLeftArmAngle] = leoState.mJointAngles[ljShoulder];
  (*obs)[svLeftArmAngleRate] = leoState.mJointSpeeds[ljShoulder];
  (*obs)[svRightHipAngle] = leoState.mJointAngles[ljHipRight];
  (*obs)[svRightHipAngleRate] = leoState.mJointSpeeds[ljHipRight];
  (*obs)[svLeftHipAngle] = leoState.mJointAngles[ljHipLeft];
  (*obs)[svLeftHipAngleRate] = leoState.mJointSpeeds[ljHipLeft];
  (*obs)[svRightKneeAngle] = leoState.mJointAngles[ljKneeRight];
  (*obs)[svRightKneeAngleRate] = leoState.mJointSpeeds[ljKneeRight];
  (*obs)[svLeftKneeAngle] = leoState.mJointAngles[ljKneeLeft];
  (*obs)[svLeftKneeAngleRate] = leoState.mJointSpeeds[ljKneeLeft];
  (*obs)[svRightAnkleAngle] = leoState.mJointAngles[ljAnkleRight];
  (*obs)[svRightAnkleAngleRate] = leoState.mJointSpeeds[ljAnkleRight];
  (*obs)[svLeftAnkleAngle] = leoState.mJointAngles[ljAnkleLeft];
  (*obs)[svLeftAnkleAngleRate] = leoState.mJointSpeeds[ljAnkleLeft];
  (*obs)[svRightToeContact] = !!(leoState.mFootContacts & LEO_FOOTSENSOR_RIGHT_TOE);
  (*obs)[svRightHeelContact] = !!(leoState.mFootContacts & LEO_FOOTSENSOR_RIGHT_HEEL);
  (*obs)[svLeftToeContact] = !!(leoState.mFootContacts & LEO_FOOTSENSOR_LEFT_TOE);
  (*obs)[svLeftHeelContact] = !!(leoState.mFootContacts & LEO_FOOTSENSOR_LEFT_HEEL);
  
  // Augmented values
  (*obs)[svStanceHipAngle] = leoState.mJointAngles[mHipStance];
  (*obs)[svStanceHipAngleRate] = leoState.mJointSpeeds[mHipStance];
  (*obs)[svSwingHipAngle] = leoState.mJointAngles[mHipSwing];
  (*obs)[svSwingHipAngleRate] = leoState.mJointSpeeds[mHipSwing];
  (*obs)[svStanceKneeAngle] = leoState.mJointAngles[mKneeStance];
  (*obs)[svStanceKneeAngleRate] = leoState.mJointSpeeds[mKneeStance];
  (*obs)[svSwingKneeAngle] = leoState.mJointAngles[mKneeSwing];
  (*obs)[svSwingKneeAngleRate] = leoState.mJointSpeeds[mKneeSwing];
  (*obs)[svStanceAnkleAngle] = leoState.mJointAngles[mAnkleStance];
  (*obs)[svStanceAnkleAngleRate] = leoState.mJointSpeeds[mAnkleStance];
  (*obs)[svSwingAnkleAngle] = leoState.mJointAngles[mAnkleSwing];
  (*obs)[svSwingAnkleAngleRate] = leoState.mJointSpeeds[mAnkleSwing];

  if (stanceLegLeft())
  {  
    (*obs)[svStanceToeContact] = (*obs)[svLeftToeContact];
    (*obs)[svStanceHeelContact] = (*obs)[svLeftHeelContact];
    (*obs)[svSwingToeContact] = (*obs)[svRightToeContact];
    (*obs)[svSwingHeelContact] = (*obs)[svRightHeelContact];
  }
  else
  {
    (*obs)[svStanceToeContact] = (*obs)[svRightToeContact];
    (*obs)[svStanceHeelContact] = (*obs)[svRightHeelContact];
    (*obs)[svSwingToeContact] = (*obs)[svLeftToeContact];
    (*obs)[svSwingHeelContact] = (*obs)[svLeftHeelContact];
  }
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
  exporter_(NULL),
  test_(0),
  time_test_(0),
  time_learn_(0),
  time0_(0)
{
}

void LeoSimEnvironment::request(ConfigurationRequest *config)
{
  ODEEnvironment::request(config);

  config->push_back(CRP("observe", "string.observe_", "Comma-separated list of state elements observed by an agent"));
  config->push_back(CRP("actuate", "string.actuate_", "Comma-separated list of action elements provided by an agent"));
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
  int ode_observation_dims = config["observation_dims"];
  int ode_action_dims = config["action_dims"];
  
  if (ode_observation_dims != svStanceHipAngle || ode_action_dims != avStanceHipTorque)
  {
    ERROR("XML configuration file does not define the right amount of observations or actions");
    return;
  }

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
  fillObserve(observeList, &observe_);

  // Get downstream observation min/max vectors from ODE ones
  Vector ode_observation_min = config["observation_min"], observation_min;
  Vector ode_observation_max = config["observation_max"], observation_max;
  observation_min.resize(observe_.size());
  observation_max.resize(observe_.size());
  for (int ii = 0; ii != observe_.size(); ++ii)
  {
    size_t idx = observe_[ii];
    if (idx >= svStanceHipAngle)
      idx = idx - svStanceHipAngle + svRightHipAngle;
    
    observation_min[ii] = ode_observation_min[idx];
    observation_max[ii] = ode_observation_max[idx];
  }
  
  config.set("observation_dims", observe_.size());
  config.set("observation_min", observation_min);
  config.set("observation_max", observation_max);

  // Parse actions
  std::string actuate = config["actuate"].str();
  std::vector<std::string> actuateList = cutLongStr(actuate);
  fillActuate(actuateList, &actuate_);
  
  // Get downstream action min/max vectors from ODE ones
  Vector ode_action_min = config["action_min"], action_min;
  Vector ode_action_max = config["action_max"], action_max;
  action_min.resize(actuate_.size());
  action_max.resize(actuate_.size());
  for (int ii = 0; ii != actuate_.size(); ++ii)
  {
    size_t idx = actuate_[ii];
    if (idx >= avStanceHipTorque)
      idx = idx - avStanceHipTorque + avRightHipTorque;
    
    action_min[ii] = ode_action_min[idx];
    action_max[ii] = ode_action_max[idx];
  }

  config.set("action_dims", actuate_.size());
  config.set("action_min", action_min);
  config.set("action_max", action_max);

  // reserve memory
  ode_obs_.v.resize(ode_observation_dims);
  ode_action_.v.resize(ode_action_dims);
}

void LeoSimEnvironment::reconfigure(const Configuration &config)
{
  ODEEnvironment::reconfigure(config);
  time_test_ = time_learn_ = time0_ = 0;
}

LeoSimEnvironment *LeoSimEnvironment::clone()
{
  return new LeoSimEnvironment(*this);
}

void LeoSimEnvironment::start(int test, Observation *obs)
{
  test_ = test;

  ODEEnvironment::start(test, &ode_obs_);
  
  bhWalk_.resetState();

  // Parse obs into CLeoState (Start with left leg being the stance leg)
  bhWalk_.fillLeoState(ode_obs_, Vector(), &leoState_);
  bhWalk_.setCurrentSTGState(&leoState_);
  bhWalk_.setPreviousSTGState(&leoState_);

  // update derived state variables
  bhWalk_.updateDerivedStateVars(&leoState_); // swing-stance switching happens here
  
  // construct new obs from CLeoState
  Vector ode_obs;
  bhWalk_.parseLeoState(leoState_, &ode_obs);
  obs->v.resize(observe_.size());
  for (int ii = 0; ii != observe_.size(); ++ii)
    obs->v[ii] = ode_obs[observe_[ii]];
    
  bhWalk_.setCurrentSTGState(NULL);

  if (exporter_)
    exporter_->open((test_?"test":"learn"), (test_?time_test_:time_learn_) != 0.0);
  time0_ = test_?time_test_:time_learn_;
}

double LeoSimEnvironment::step(const Action &action, Observation *obs, double *reward, int *terminal)
{
  double &time = test_?time_test_:time_learn_;
  bhWalk_.setCurrentSTGState(&leoState_);
  
  // Calculate actuation from action
  Vector actuation((size_t)avNumActions);
  actuation.fill(std::numeric_limits<double>::quiet_NaN());
  
  if (action.size() != actuate_.size())
  {
    ERROR("Expected action dimension " << actuate_.size() << ", got " << action.size());
    throw bad_param("environment/leosim:actuate");
  }
  
  for (size_t ii=0; ii != actuate_.size(); ++ii)
    actuation[actuate_[ii]] = action[ii];
    
  if (isnan(actuation[avLeftArmTorque]))
    actuation[avLeftArmTorque] = bhWalk_.grlAutoActuateArm();
    
  if (isnan(actuation[avStanceKneeTorque]))
    actuation[avStanceKneeTorque] = bhWalk_.grlAutoActuateKnee();
  
  if (bhWalk_.stanceLegLeft())
  {
    if (isnan(actuation[avRightHipTorque])) actuation[avRightHipTorque] = actuation[avSwingHipTorque];
    if (isnan(actuation[avLeftHipTorque])) actuation[avLeftHipTorque] = actuation[avStanceHipTorque];
    if (isnan(actuation[avRightKneeTorque])) actuation[avRightKneeTorque] = actuation[avSwingKneeTorque];
    if (isnan(actuation[avLeftKneeTorque])) actuation[avLeftKneeTorque] = actuation[avStanceKneeTorque];
    if (isnan(actuation[avRightAnkleTorque])) actuation[avRightAnkleTorque] = actuation[avSwingAnkleTorque];
    if (isnan(actuation[avLeftAnkleTorque])) actuation[avLeftAnkleTorque] = actuation[avStanceAnkleTorque];
  }
  else
  {
    if (isnan(actuation[avRightHipTorque])) actuation[avRightHipTorque] = actuation[avStanceHipTorque];
    if (isnan(actuation[avLeftHipTorque])) actuation[avLeftHipTorque] = actuation[avSwingHipTorque];
    if (isnan(actuation[avRightKneeTorque])) actuation[avRightKneeTorque] = actuation[avStanceKneeTorque];
    if (isnan(actuation[avLeftKneeTorque])) actuation[avLeftKneeTorque] = actuation[avSwingKneeTorque];
    if (isnan(actuation[avRightAnkleTorque])) actuation[avRightAnkleTorque] = actuation[avStanceAnkleTorque];
    if (isnan(actuation[avLeftAnkleTorque])) actuation[avLeftAnkleTorque] = actuation[avSwingAnkleTorque];
  }

  Vector actionAnkles;
  bhWalk_.grlAutoActuateAnkles(&actionAnkles);

  if (isnan(actuation[avRightAnkleTorque]))
    actuation[avRightAnkleTorque] = actionAnkles[0];

  if (isnan(actuation[avLeftAnkleTorque]))
    actuation[avLeftAnkleTorque] = actionAnkles[1];
    
  for (size_t ii=0; ii != avStanceHipTorque; ++ii)
    if (isnan(actuation[ii]))
    {
      ERROR("Could not derive actuation index " << ii);
      throw bad_param("environment/leosim:actuate");
    }
    
  // concatenation happens in the order of <actionvar> definitions in an xml file
  // shoulder, right hip, left hip, right knee, left knee, right ankle, left ankle
  ode_action_.v = actuation.head(avStanceHipTorque);

  bhWalk_.setPreviousSTGState(&leoState_);
  TRACE("ode action = " << ode_action_);
  double tau = ODEEnvironment::step(ode_action_, &ode_obs_, reward, terminal);
  TRACE("ode observation = " << ode_obs_);

  // Filter joint speeds
  // Parse obs into CLeoState
  bhWalk_.fillLeoState(ode_obs_, ode_action_, &leoState_);
  bhWalk_.setCurrentSTGState(&leoState_);

  // update derived state variables
  bhWalk_.updateDerivedStateVars(&leoState_);

  // construct new obs from CLeoState
  Vector ode_obs;
  bhWalk_.parseLeoState(leoState_, &ode_obs);
  obs->v.resize(observe_.size());
  for (int ii = 0; ii != observe_.size(); ++ii)
    obs->v[ii] = ode_obs[observe_[ii]];

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
  
  Vector s0v, s1v, av;
  toVector(s0, s0v);
  toVector(s1, s1v);
  toVector(a, av);
  
  if (exporter_)
    exporter_->write({grl::VectorConstructor(time), s0v, s1v,
                      av, grl::VectorConstructor(*reward), grl::VectorConstructor(*terminal)
                     });

  time += tau;

  return tau;
}

void LeoSimEnvironment::report(std::ostream &os) const
{
  const double &time  = test_?time_test_ :time_learn_;
  os << bhWalk_.getProgressReport(time-time0_);
}

void LeoSimEnvironment::fillObserve( const std::vector<std::string> &observeList,
                                     IndexVector *out) const
{
  out->resize(observeList.size());
  for (size_t ii = 0; ii != out->size(); ++ii)
  {
    std::string str = observeList[ii];
    for (size_t jj=0; jj != str.size(); ++jj)
      str[jj] = tolower(str[jj]);
    
           if (str == "torso.angle")           (*out)[ii] = svTorsoAngle;
      else if (str == "torso.anglerate")       (*out)[ii] = svTorsoAngleRate;
      else if (str == "shoulder.angle")        (*out)[ii] = svLeftArmAngle;
      else if (str == "shoulder.anglerate")    (*out)[ii] = svLeftArmAngleRate;
      else if (str == "righthip.angle")        (*out)[ii] = svRightHipAngle;
      else if (str == "righthip.anglerate")    (*out)[ii] = svRightHipAngleRate;
      else if (str == "lefthip.angle")         (*out)[ii] = svLeftHipAngle;
      else if (str == "lefthip.anglerate")     (*out)[ii] = svLeftHipAngleRate;
      else if (str == "rightknee.angle")       (*out)[ii] = svRightKneeAngle;
      else if (str == "rightknee.anglerate")   (*out)[ii] = svRightKneeAngleRate;
      else if (str == "leftknee.angle")        (*out)[ii] = svLeftKneeAngle;
      else if (str == "leftknee.anglerate")    (*out)[ii] = svLeftKneeAngleRate;
      else if (str == "rightankle.angle")      (*out)[ii] = svRightAnkleAngle;
      else if (str == "rightankle.anglerate")  (*out)[ii] = svRightAnkleAngleRate;
      else if (str == "leftankle.angle")       (*out)[ii] = svLeftAnkleAngle;
      else if (str == "leftankle.anglerate")   (*out)[ii] = svLeftAnkleAngleRate;
      else if (str == "righttoe.contact")      (*out)[ii] = svRightToeContact;
      else if (str == "rightheel.contact")     (*out)[ii] = svRightHeelContact;
      else if (str == "lefttoe.contact")       (*out)[ii] = svLeftToeContact;
      else if (str == "leftheel.contact")      (*out)[ii] = svLeftHeelContact;
      
      else if (str == "stancehip.angle")       (*out)[ii] = svStanceHipAngle;
      else if (str == "stancehip.anglerate")   (*out)[ii] = svStanceHipAngleRate;
      else if (str == "swinghip.angle")        (*out)[ii] = svSwingHipAngle;
      else if (str == "swinghip.anglerate")    (*out)[ii] = svSwingHipAngleRate;
      else if (str == "stanceknee.angle")      (*out)[ii] = svStanceKneeAngle;
      else if (str == "stanceknee.anglerate")  (*out)[ii] = svStanceKneeAngleRate;
      else if (str == "swingknee.angle")       (*out)[ii] = svSwingKneeAngle;
      else if (str == "swingknee.anglerate")   (*out)[ii] = svSwingKneeAngleRate;
      else if (str == "stanceankle.angle")     (*out)[ii] = svStanceAnkleAngle;
      else if (str == "stanceankle.anglerate") (*out)[ii] = svStanceAnkleAngleRate;
      else if (str == "swingankle.angle")      (*out)[ii] = svSwingAnkleAngle;
      else if (str == "swingankle.anglerate")  (*out)[ii] = svSwingAnkleAngleRate;
      else if (str == "stancetoe.contact")     (*out)[ii] = svStanceToeContact;
      else if (str == "stanceheel.contact")    (*out)[ii] = svStanceHeelContact;
      else if (str == "swingtoe.contact")      (*out)[ii] = svSwingToeContact;
      else if (str == "swingheel.contact")     (*out)[ii] = svSwingHeelContact;

      else
      {
        ERROR("Requested unregistered field '" << str << "'");
        throw bad_param("leosim:observe");
      }
  }
}

void LeoSimEnvironment::fillActuate( const std::vector<std::string> &actuateList,
                                     IndexVector *out) const
{
  out->resize(actuateList.size());
  for (size_t ii = 0; ii != out->size(); ++ii)
  {
    std::string str = actuateList[ii];
    for (size_t jj=0; jj != str.size(); ++jj)
      str[jj] = tolower(str[jj]);

           if (str == "shoulder.torque")   (*out)[ii] = avLeftArmTorque;
      else if (str == "righthip.torque")   (*out)[ii] = avRightHipTorque;
      else if (str == "lefthip.torque")    (*out)[ii] = avLeftHipTorque;
      else if (str == "rightknee.torque")  (*out)[ii] = avRightKneeTorque;
      else if (str == "leftknee.torque")   (*out)[ii] = avLeftKneeTorque;
      else if (str == "rightankle.torque") (*out)[ii] = avRightAnkleTorque;
      else if (str == "leftankle.torque")  (*out)[ii] = avLeftAnkleTorque;

      else if (str == "stancehip.torque")   (*out)[ii] = avStanceHipTorque;
      else if (str == "swinghip.torque")    (*out)[ii] = avSwingHipTorque;
      else if (str == "stanceknee.torque")  (*out)[ii] = avStanceKneeTorque;
      else if (str == "swingknee.torque")   (*out)[ii] = avSwingKneeTorque;
      else if (str == "stanceankle.torque") (*out)[ii] = avStanceAnkleTorque;
      else if (str == "swingankle.torque")  (*out)[ii] = avSwingAnkleTorque;

      else
      {
        ERROR("Requested unregistered field '" << str << "'");
        throw bad_param("leosim:actuate");
      }
  }
}
