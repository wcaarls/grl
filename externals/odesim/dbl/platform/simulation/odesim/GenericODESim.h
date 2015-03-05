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

#ifndef __GENERICSIMULATION_H_INCLUDED
#define __GENERICSIMULATION_H_INCLUDED

#include <STGODESim.h>
#include <GenericState.h>

#include <Log2.h>
#include "ODELogging.h"

class CODEException: public std::exception
{
  protected:
    std::string mMsg;
  
  public:
    CODEException(const char *msg)
    {
      mMsg = msg;
    }
    virtual ~CODEException() throw ()
    {
    }

    virtual const char* what() const throw()
    {
      return mMsg.c_str();
    }
};       

class IGenericSensor
{
  private:
    std::string mName;

  public:
    IGenericSensor(const std::string &name) : mName(name) { }
    virtual ~IGenericSensor() { }
    
    virtual double getValue() = 0;
    virtual const std::string &getName() const { return mName; }
};

class CGenericJointSensor : public IGenericSensor, public CODELoggable
{
  private:
    enum ESensorType {SensorTypeAngle, SensorTypeAngleRate, SensorTypePosition, SensorTypePositionRate, SensorTypeTorque, SensorTypeVoltage};

    CODEJoint *mJoint;
    ESensorType mSensorType;

  public:
    CGenericJointSensor(const std::string &name, CODEJoint *joint, const std::string &type) : IGenericSensor(name), mJoint(joint)
    {
      if (type == "angle")
        mSensorType = SensorTypeAngle;
      else if (type == "anglerate")
        mSensorType = SensorTypeAngleRate;
      else if (type == "position")
        mSensorType = SensorTypePosition;
      else if (type == "positionrate")
        mSensorType = SensorTypePositionRate;
      else if (type == "torque")
        mSensorType = SensorTypeTorque;
      else if (type == "voltage")
        mSensorType = SensorTypeVoltage;
      else
        mLogErrorLn("Unknown sensor type " << type);
    }
    virtual ~CGenericJointSensor() { }

    virtual double getValue()
    {
      switch (mSensorType)
      {
        case SensorTypeAngle:
          return mJoint->getAngle();
        case SensorTypeAngleRate:
          return mJoint->getAngleRate();
        case SensorTypePosition:
          return mJoint->getPosition();
        case SensorTypePositionRate:
          return mJoint->getPositionRate();
        case SensorTypeTorque:
          return mJoint->getMotor(0)->getTorque();
        case SensorTypeVoltage:
          return mJoint->getMotor(0)->getVoltage();
        default:
          return 0;	// To avoid compiler warning
      }
    }
};

class CGenericBodySensor : public IGenericSensor, public CODELoggable
{
  private:
    enum ESensorType {SensorTypePosition, SensorTypeOrientation, SensorTypeVelocity};

    CODEBody *mBody;
    ESensorType mSensorType;
    int mAxis;

  public:
    CGenericBodySensor(const std::string &name, CODEBody *body, const std::string &type, const std::string &axis) : IGenericSensor(name), mBody(body)
    {
      if (type == "position")
        mSensorType = SensorTypePosition;
      else if (type == "orientation")
        mSensorType = SensorTypeOrientation;
      else if (type == "velocity")
        mSensorType = SensorTypeVelocity;
      else
        mLogErrorLn("Unknown sensor type " << type);

      if (axis == "x")
        mAxis = 0;
      else if (axis == "y")
        mAxis = 1;
      else if (axis == "z")
        mAxis = 2;
      else if (axis == "w")
        mAxis = 3;
      else
        mLogErrorLn("Unknown axis " << mAxis);
    }
    virtual ~CGenericBodySensor() { }
    
    virtual double getValue()
    {
      const dReal *vec;

      switch (mSensorType)
      {
        case SensorTypePosition:
          if (mAxis > 2)
          {
            mLogErrorLn("Unsupported axis");
            return 0;
          }
          
          vec = dBodyGetPosition(mBody->id());
          return vec[mAxis];
        case SensorTypeOrientation:
          vec = dBodyGetQuaternion(mBody->id());

		  // Quaternion is stored in wxyz order
		  if (mAxis == 3)
	          return vec[0];
		  else
	          return vec[mAxis+1];
        case SensorTypeVelocity:
          if (mAxis > 2)
          {
            mLogErrorLn("Unsupported axis");
            return 0;
          }
          
          vec = dBodyGetLinearVel(mBody->id());
          return vec[mAxis];
        default:
          return 0; // To avoid compiler warning
      }
    }
};

class CGenericDistanceSensor : public IGenericSensor, public CODELoggable
{
  private:
    CODEBody *mBody, *mRelBody;
    int mAxis;

  public:
    CGenericDistanceSensor(const std::string &name, CODEBody *body, CODEBody *relBody, const std::string &axis) : IGenericSensor(name), mBody(body), mRelBody(relBody), mAxis(0)
    {
      if (axis == "x")
        mAxis = 0;
      else if (axis == "y")
        mAxis = 1;
      else if (axis == "z")
        mAxis = 2;
      else if (axis == "azimuth")
        mAxis = 3;
      else if (axis == "elevation")
        mAxis = 4;
      else if (axis == "distance")
        mAxis = 5;
      else
        mLogErrorLn("Unknown axis " << axis);
    }
    virtual ~CGenericDistanceSensor() { }

    virtual double getValue()
    {
      dVector3 vecAbs, vec;
      dBodyCopyPosition(mBody->id(), vecAbs);
      dBodyGetPosRelPoint(mRelBody->id(), vecAbs[0], vecAbs[1], vecAbs[2], vec);

      switch (mAxis)
      {
        case 3: // azimuth
          return atan2(vec[1], vec[0]);
        case 4: // elevation
          return atan2(vec[2], sqrt(vec[0]*vec[0]+vec[1]*vec[1]));
        case 5: // distance
          return sqrt(vec[0]*vec[0]+vec[1]*vec[1]+vec[2]*vec[2]);
        default:
          return vec[mAxis];
      }
    }
};

class CGenericGeomSensor : public IGenericSensor, public CODELoggable
{
  private:
    enum ESensorType {SensorTypeContact};

    CODEGeom *mGeom;
    ESensorType mSensorType;

  public:
    CGenericGeomSensor(const std::string &name, CODEGeom *geom, const std::string &type) : IGenericSensor(name), mGeom(geom)
    {
      if (type == "contact")
        mSensorType = SensorTypeContact;
      else
        mLogErrorLn("Unknown sensor type " << type);
    }
    virtual ~CGenericGeomSensor() { }

    virtual double getValue()
    {
      switch (mSensorType)
      {
        case SensorTypeContact:
          return mGeom->getContacts().size() > 0;
        default:
          return 0;	// To avoid compiler warning
      }
    }
};

class IGenericActuator
{
  private:
    std::string mName;
    double mAction;

  public:
    IGenericActuator(const std::string &name) : mName(name), mAction(0) { }
    virtual ~IGenericActuator() { }
    
    virtual void setValue(double value) = 0;
    virtual void activate() = 0;
    virtual const std::string &getName() const { return mName; }

    virtual void setAction(double value) { mAction = value; }
    virtual double getAction() { return mAction; }
};

class CGenericJointActuator : public IGenericActuator, public CODELoggable
{
  private:
    CODEJointMotor *mMotor;
    ESTGActuationMode mActuationMode;
    double mValue;

  public:
    CGenericJointActuator(const std::string &name, CODEJointMotor *motor, ESTGActuationMode mode) : IGenericActuator(name), mMotor(motor), mActuationMode(mode), mValue(0) { }
    virtual ~CGenericJointActuator() { }

    virtual void setValue(double value)
    {
      mValue = value;
    }

    virtual void activate()
    {
      switch (mActuationMode)
      {
        case amTorque:
          mMotor->setTorque(mValue);
          break;
        case amVoltage:
          mMotor->setVoltage(mValue);
          break;
        case amForce:
          mMotor->setForce(mValue);
          break;
        default:
          mLogErrorLn("Unimplemented actuation mode " << mActuationMode);
          break;
      }
    }
};

class CGenericBodyActuator : public IGenericActuator, public CODELoggable
{
  private:
    CODEExtForce *mExtForce;
    int mAxis;
    double mValue;

  public:
    CGenericBodyActuator(const std::string &name, CODEBodyAnchor *anchor, const std::string &axis) : IGenericActuator(name), mAxis(0), mValue(0)
    {
      if (axis == "x")
        mAxis = 0;
      else if (axis == "y")
        mAxis = 1;
      else if (axis == "z")
        mAxis = 2;
      else
        mLogErrorLn("Unknown axis " << axis);

      mExtForce = new CODEExtForce(anchor->body()->object());
      mExtForce->setBodyName(anchor->body()->name());
      mExtForce->setPosition(anchor->x(), anchor->y(), anchor->z());
      mExtForce->init();
      anchor->body()->object()->addExternalForce(mExtForce);
    }
    
    virtual ~CGenericBodyActuator() { }

    virtual void setValue(double value)
    {
      mValue = value;
    }

    virtual void activate()
    {
      double force[] = {0, 0, 0};
      force[mAxis] = mValue;
      mExtForce->setForce(force[0], force[1], force[2]);
    }
};

class CGenericODESim: public CSTGODESim<GenericState>, public ISTGGenericSensing, public ISTGGenericActuation, public CODELoggable
{
  protected:
    std::vector<IGenericSensor*> mSensors;
    std::vector<IGenericActuator*> mActuators;
    int mStateStartIndex, mActionStartIndex;

  public:
    CGenericODESim(const std::string &name, int stateStartIndex=0, int actionStartIndex=0) :
      CSTGODESim<GenericState>(name),
      mStateStartIndex(stateStartIndex), mActionStartIndex(actionStartIndex) { }
    virtual ~CGenericODESim() { }

    // CSTGODESim
    virtual bool init();
    virtual void fillState();
    virtual int  activateActions(const uint64_t& stateID);
    virtual void resetActuationValues();
    virtual void setJointValue(int idx, double value);
    virtual void setJointTorque(int idx, double value);
    virtual void setJointForce(int idx, double value);
    virtual void setJointVoltage(int idx, double value);
    virtual void setJointAction(int idx, double value);

    // ISTGGenericSensing
    virtual int getStateIndex(const std::string &name);

    // ISTGGenericActuation
    virtual int getActionIndex(const std::string &name);

    virtual void setInitialCondition(long int seed=0)
    {
      CODEInitializationExpression::randomInit(seed);
    }
};

#endif /* __GENERICSIMULATION_H_INCLUDED */
