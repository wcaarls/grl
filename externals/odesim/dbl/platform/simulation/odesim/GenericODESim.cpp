/*
 *    Version of CSTGODESim that uses the GenericState interface.
 *    Copyright (C) 2012 Wouter Caarls (DBL)
 *
 *    This program is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU Lesser General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public License
 *    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <GenericODESim.h>

using std::cout;
using std::endl;

void ODEMessageFunction(int errnum, const char *msg, va_list ap)
{
  char buf[1024];
  vsnprintf(buf, 1024, msg, ap);

  if (errnum)
    logErrorLn(CLog2("ode"), "Error " << errnum << ": " << buf);
  else
    logErrorLn(CLog2("ode"), buf);

  throw CODEException(buf);
}

bool CGenericODESim::init()
{
  dSetErrorHandler(ODEMessageFunction);
  dSetDebugHandler(ODEMessageFunction);
  dSetMessageHandler(ODEMessageFunction);

  mSensors.clear();
  mActuators.clear();
  return CSTGODESim<GenericState>::init();
}

void CGenericODESim::fillState()
{
  for (unsigned int ii=0; ii != mSensors.size(); ++ii)
  {
    mLogCrawlLn("state[" << mStateStartIndex+ii << "] = " << mSensors[ii]->getValue());
    mState.var[mStateStartIndex+ii] = mSensors[ii]->getValue();
  }
  for (unsigned int ii=0; ii != mActuators.size(); ++ii)
  {
    mState.prevAction[mActionStartIndex+ii] = mActuators[ii]->getAction();
  }
}

int CGenericODESim::activateActions(const uint64_t& stateID)
{
  for (unsigned int ii=0; ii != mActuators.size(); ++ii)
    mActuators[ii]->activate();
  return 0;	// Success
}

void CGenericODESim::resetActuationValues()
{
  for (unsigned int ii=0; ii != mActuators.size(); ++ii)
  {
    mActuators[ii]->setValue(0);
    mActuators[ii]->setAction(0);
  }
}

void CGenericODESim::setJointValue(int idx, double value)
{
  if (idx >= mActionStartIndex && idx < ((int)mActuators.size())-mActionStartIndex)
  {
    mLogCrawlLn("action[" << idx << "] = " << value);
    mActuators[idx-mActionStartIndex]->setValue(value);
  }
  else
    mLogErrorLn("Action index out of bounds");
}

void CGenericODESim::setJointTorque(int idx, double value)
{
	setJointValue(idx, value);
}

void CGenericODESim::setJointForce(int idx, double value)
{
	setJointValue(idx, value);
}

void CGenericODESim::setJointVoltage(int idx, double value)
{
	setJointValue(idx, value);
}

void CGenericODESim::setJointAction(int idx, double value)
{
  if (idx >= mActionStartIndex && idx < ((int)mActuators.size())-mActionStartIndex)
  {
    mActuators[idx-mActionStartIndex]->setAction(value);
  }
  else
    mLogErrorLn("Action index out of bounds");
}

int CGenericODESim::getActionIndex(const std::string &name)
{
  // Check previously defined actuators
  for (unsigned int ii=0; ii != mActuators.size(); ++ii)
    if (mActuators[ii]->getName() == name)
      return mActionStartIndex + ii;

  // Acquire sim access
  CODESimAccess simAccess(&mSim);

  std::string temp = name;

  std::string objectName = temp.substr(0, temp.find('.'));
  temp = temp.substr(temp.find('.')+1, temp.npos);
  std::string jointName  = temp.substr(0, temp.find('.'));
  temp = temp.substr(temp.find('.')+1, temp.npos);
  std::string motorName  = temp.substr(0, temp.find('.'));
  if (temp.find('.') != temp.npos)
    temp = temp.substr(temp.find('.')+1, temp.npos);
  else
    temp = "";
  std::string modeName = temp;

  if (modeName == "")
  {
    modeName = motorName;
    motorName = "";
  }

  CODEObject *object = simAccess.resolveObject(objectName);
  if (!object)
  {
    mLogErrorLn("Couldn't find object " << objectName << " while resolving action " << name);
    return -1;
  }

  CODEJoint  *joint  = object->resolveJoint(jointName);
  if (!joint)
  {
    CODEBody *body = object->resolveBody(jointName);
    if (!body)
    {
      mLogErrorLn("Couldn't find joint " << jointName << " of object " << objectName << " while resolving action " << name);
      return -1;
    }
    
    const std::vector<CODEBodyAnchor*>& anchors = body->getAnchors();
    CODEBodyAnchor *anchor=NULL;
    
    for (size_t ii=0; ii < anchors.size(); ++ii)
      if (anchors[ii]->name() == motorName)
        anchor = anchors[ii];
        
    if (anchor == NULL)
    {
      mLogErrorLn("Couldn't find anchor " << motorName << " of body " << jointName << " of object " << objectName << " while resolving action " << name);
      return -1;
    }
    
    CGenericBodyActuator *actuator = new CGenericBodyActuator(name, anchor, modeName);
    mActuators.push_back(actuator);
    
    return mActionStartIndex + mActuators.size()-1;
  }

  int motorAxis = 0;

  if (motorName == "axis2")
    motorAxis = 1;
  else if (motorName == "axis3")
    motorAxis = 2;

  CGenericJointActuator *actuator =
      new CGenericJointActuator(name, joint->getMotor(motorAxis), getActuationModeByName(modeName));
  mActuators.push_back(actuator);

  return mActionStartIndex + mActuators.size()-1;
}

int CGenericODESim::getStateIndex(const std::string &name)
{
  // Check previously defined sensors
  for (unsigned int ii=0; ii != mSensors.size(); ++ii)
    if (mSensors[ii]->getName() == name)
      return mStateStartIndex + ii;

  // Acquire sim access
  CODESimAccess simAccess(&mSim);

  std::string temp = name;

  std::string objectName = temp.substr(0, temp.find('.'));
  temp = temp.substr(temp.find('.')+1, temp.npos);
  std::string memberName  = temp.substr(0, temp.find('.'));
  temp = temp.substr(temp.find('.')+1, temp.npos);

  CODEObject *object = simAccess.resolveObject(objectName);
  if (!object)
  {
    mLogErrorLn("Couldn't find object " << objectName << " while resolving state " << name);
    return -1;
  }

  if (temp.find('@') != temp.npos)
  {
    // Relative position
    std::string axisName  = temp.substr(0, temp.find('@'));
    temp = temp.substr(temp.find('@')+1, temp.npos);
    std::string relObjectName  = temp.substr(0, temp.find('.'));
    temp = temp.substr(temp.find('.')+1, temp.npos);
    std::string relBodyName  = temp;

    CODEBody *body = object->resolveBody(memberName);
    if (!body)
    {
      mLogErrorLn("Couldn't find body " << objectName << "." << memberName << " while resolving state " << name);
      return -1;
    }

    CODEObject *relObject = simAccess.resolveObject(relObjectName);
    if (!relObject)
    {
      mLogErrorLn("Couldn't find object " << relObjectName << " while resolving state " << name);
      return -1;
    }

    CODEBody *relBody = relObject->resolveBody(relBodyName);
    if (!relBody)
    {
      mLogErrorLn("Couldn't find body " << relObjectName << "." << relBodyName << " while resolving state " << name);
      return -1;
    }

    CGenericDistanceSensor *sensor = new CGenericDistanceSensor(name, body, relBody, axisName);
    mSensors.push_back(sensor);
    return mStateStartIndex + mSensors.size()-1;
  }
  else
  {
    std::string subMemberName  = temp.substr(0, temp.find('.'));
    if (temp.find('.') != temp.npos)
      temp = temp.substr(temp.find('.')+1, temp.npos);
    else
      temp = "";
    std::string subSubMemberName = temp;

    CODEObject *object = simAccess.resolveObject(objectName);
    if (!object)
    {
      mLogErrorLn("Couldn't find object " << objectName << " while resolving state " << name);
      return -1;
    }

    CODEJoint *joint = object->resolveJoint(memberName);
    if (joint)
    {
      // Joint position
      CGenericJointSensor *sensor = new CGenericJointSensor(name, joint, subMemberName);
      mSensors.push_back(sensor);
      return mStateStartIndex + mSensors.size()-1;
    }

    CODEBody *body = object->resolveBody(memberName);
    if (body)
    {
      // Body position
      if (subSubMemberName == "")
      {
        subSubMemberName = subMemberName;
        subMemberName = "position";
      }

      CGenericBodySensor *sensor = new CGenericBodySensor(name, body, subMemberName, subSubMemberName);
      mSensors.push_back(sensor);
      return mStateStartIndex + mSensors.size()-1;
    }

    CODEGeom *geom = object->resolveGeom(memberName);
    if (geom)
    {
      // Contact
      CGenericGeomSensor *sensor = new CGenericGeomSensor(name, geom, subMemberName);
      mSensors.push_back(sensor);
      return mStateStartIndex + mSensors.size()-1;
    }

    mLogErrorLn("Couldn't find member " << memberName << " of object " << objectName << " while resolving state " << name);
    return -1;
  }
}
