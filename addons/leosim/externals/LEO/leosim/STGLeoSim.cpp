/*
 * STGLeoSim.cpp
 *
 *  Created on: Apr 9, 2009
 *      Author: Erik Schuitema
 */

#include "STGLeoSim.h"

// ******************************************************************************* //
// ********************************* CSTGLeoSim ********************************** //
// ******************************************************************************* //

CSTGLeoSim::CSTGLeoSim():
  mLog("leosim")
{
  resetBindData();
}

double CSTGLeoSim::getJointVoltage(int jointIndex)
{
  return mMotorJointVoltages[jointIndex];
}

void CSTGLeoSim::setJointVoltage(int jointIndex, double voltage)
{
	if (jointIndex < ljNumDynamixels)
	{
		// Don't allow voltages larger than the maximum. In the real robot, the Dynamixel actuation functions limit the voltage.
		double voltageLimit = getJointMaxVoltage(jointIndex);
		double clippedVoltage = std::max(std::min(voltage, voltageLimit), -voltageLimit);
    mMotorJointVoltages[jointIndex] = clippedVoltage;
	}
}

double CSTGLeoSim::getJointMaxVoltage(int jointIndex)
{
	// Leo runs on a fixed voltage power supply, and all motors are RX-28's
	// Voltages are limited below the supply voltage due to temperature compensation (see macro definition)
	if (jointIndex < ljNumDynamixels)
		return LEO_MAX_DXL_VOLTAGE;
	else
		return 0;
}

void CSTGLeoSim::fillState(CLeoState &state)
{
  // Get foot contact data
  state.mFootContacts			= 0;
  for (int iFoot=0; iFoot<lfNumFootContacts; iFoot++)
  {
    // If a foot geom collides with *anything*, count it as foot contact
    if (mFootContactGeoms[iFoot] != NULL)
      if (mFootContactGeoms[iFoot]->getContacts().size() > 0)
        state.mFootContacts |= 1 << iFoot;
    state.mFootSensors[iFoot]	= 0;	// No data yet
  }
}

void CSTGLeoSim::resetBindData()
{
  for (int i=0; i<ljNumDynamixels; i++)
    mMotorJoints[i] = NULL;
  for (int i=0; i<lpNumParts; i++)
    mBodyParts[i] = NULL;
  for (int i=0; i<lfNumFootContacts; i++)
    mFootContactGeoms[i] = NULL;
}

const char* CSTGLeoSim::getFootContactName(int footContactIndex)
{
  switch (footContactIndex)
  {
    case lfToeRight:	return "toeright"; break;
    case lfToeLeft:		return "toeleft"; break;
    case lfHeelRight:	return "heelright"; break;
    case lfHeelLeft:	return "heelleft"; break;
    default:
      return NULL;
  }
}

const char* CSTGLeoSim::getBodyPartName(int partIndex)
{
  switch (partIndex)
  {
    case lpTorso:			return "torso"; break;
    case lpUpperLegLeft:	return "upperlegleft"; break;
    case lpUpperLegRight:	return "upperlegright"; break;
    case lpLowerLegLeft:	return "lowerlegleft"; break;
    case lpLowerLegRight:	return "lowerlegright"; break;
    case lpFootLeft:		return "footleft"; break;
    case lpFootRight:		return "footright"; break;
    case lpArm:				return "arm"; break;
    default:
      return NULL;
  }
}

// Bind elements (joints, bodies,..) from XML to the expected ones for Leo
bool CSTGLeoSim::bindRobot(CODESim *sim)
{
  bool result = true;

  // Reset bind data
  resetBindData();

  // Acquire sim access
  CODESimAccess simAccess(sim);

  // Acquire "robot" object pointer
  CODEObject* robot = simAccess.resolveObject("robot");
  if (robot == NULL)
    result = false;
  else
  {
    // Request hinge joint pointers
    for (int i=0; i<ljNumDynamixels; i++)
    {
      CODEJoint* joint = robot->resolveJoint(getJointName(i));
      if (joint != NULL)
        if (joint->getType() == dJointTypeHinge)
          mMotorJoints[i] = (CODEHingeJoint*)joint;

      if (mMotorJoints[i] == NULL)
      {
        mLogWarningLn("Could not resolve joint \"" << getJointName(i) << "\" from current XML configuration!");
        //result = false;
      }
    }
    // Request body pointers
    for (int i=0; i<lpNumParts; i++)
    {
      mBodyParts[i] = robot->resolveBody(getBodyPartName(i));
      if (mBodyParts[i] == NULL)
      {
        mLogWarningLn("Could not resolve body \"" << getBodyPartName(i) << "\" from current XML configuration!");
        //result = false;
      }
    }
    // Request foot contact geoms
    for (int i=0; i<lfNumFootContacts; i++)
    {
      mFootContactGeoms[i] = robot->resolveGeom(getFootContactName(i));
      if (mFootContactGeoms[i] == NULL)
      {
        mLogWarningLn("Could not resolve geom \"" << getFootContactName(i) << "\" from current XML configuration!");
        //result = false;
      }
    }
    // Request error LEDs
    mErrorLEDs[0] = mBodyParts[lpTorso]->resolveDrawingObject("errorled1");
    mErrorLEDs[1] = mBodyParts[lpTorso]->resolveDrawingObject("errorled2");
    mErrorLEDs[2] = mBodyParts[lpTorso]->resolveDrawingObject("errorled3");
  }


  // Sim access is released when simAccess goes out of scope
  return result;
}
