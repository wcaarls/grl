#include <XMLConfiguration.h>

#include <grl/environments/odesim/leosim.h>


using namespace grl;

REGISTER_CONFIGURABLE(LeoSimEnvironment)

void LeoSimEnvironment::request(ConfigurationRequest *config)
{
  ODEEnvironment::request(config);
}

void LeoSimEnvironment::configure(Configuration &config)
{
  ODEEnvironment::configure(config);
  
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
  ODEEnvironment::start(test, obs);
/*
  // TODO: Parse obs into CLeoState
  CLeoState leoState;
  siTorsoAngle,
  siTorsoAngleRate,
  siHipStanceAngle,
  siHipStanceAngleRate,
  siHipSwingAngle,
  siHipSwingAngleRate,
  siKneeStanceAngle,
  siKneeStanceAngleRate,
  siKneeSwingAngle,
  siKneeSwingAngleRate,

//  leoState.mFootContacts = 
  leoState.mJointAngles[ljTorso] = *(obs)[CLeoBhWalkSym::siTorsoAngle];
  leoState.mJointAngles[ljHipLeft] = *(obs)[CLeoBhWalkSym::siTorsoAngle];
  
  // TODO: update derived state variables (LeoBhWalkSym.cpp:281)
//  uint64_t newStateTime = getSTG()->getAbsTime();
//  bhWalk.reset(newStateTime);
  bhWalk.updateDerivedStateVars(leoState);
*/
  // TODO: construct new obs from CLeoState (policy.cpp:162)
}

double LeoSimEnvironment::step(const Vector &action, Vector *obs, double *reward, int *terminal)
{
  // TODO: auto actuate unlearned joints to find complete action vector (LeoBhWalkSym.cpp:880)

  ODEEnvironment::step(action, obs, reward, terminal);

  // TODO: Filter joint speeds (STGLeoSim.cpp:275)
  
  // TODO: Parse obs into CLeoState
  
  // TODO: update derived state variables (LeoBhWalkSym.cpp:281)

  // TODO: construct new obs from CLeoState (policy.cpp:162)

  // TODO: Determine reward and termination (LeoBhWalkSym.cpp:693)
}
