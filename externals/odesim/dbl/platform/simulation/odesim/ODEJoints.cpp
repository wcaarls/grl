/*
 *    Copyright (C) 2012 Erik Schuitema (DBL)
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

#include "ODEJoints.h"
#include "ODEObjects.h"

CODEJoint::CODEJoint(CODEObject *parentObject):
	mID(0),
	mName("nameless-joint"),
	mpObject(parentObject)
{
	mpBodyAnchors[0] = NULL;
	mpBodyAnchors[1] = NULL;
}

CODEJoint::~CODEJoint()
{
	if (mID)
		dJointDestroy (mID);
}

std::string CODEJoint::getTypeName() const
{
	switch(getType())
	{
		case dJointTypeNone:
			return "none";
		case dJointTypeBall:
			return "ball";
		case dJointTypeHinge:
			return "hinge";
		case dJointTypeSlider:
			return "slider";
		case dJointTypeContact:
			return "contact";
		case dJointTypeUniversal:
			return "universal";
		case dJointTypeHinge2:
			return "hinge2";
		case dJointTypeFixed:
			return "fixed";
		case dJointTypeNull:
			return "null";
		case dJointTypeAMotor:
			return "Amotor";
		case dJointTypeLMotor:
			return "Lmotor";
		case dJointTypePlane2D:
			return "plane2D";
		case dJointTypePR:
			return "PR";
		case dJointTypePU:
			return "PU";
		case dJointTypePiston:
			return "piston";
		default:
			return "unknown";
	}
}

bool CODEJoint::assertAnchors()
{
	return (mpBodyAnchors[0] != NULL) && (mpBodyAnchors[1] != NULL);
}

bool CODEJoint::readConfig(const CConfigSection &configSection)
{
	bool configresult=true;

	// Read name
	configresult &= mLogAssert(configSection.get("name", &mName, "nameless-joint"));

	// Read anchors
	std::string bodyName, anchorName;
	std::string anchorNodeNames[2] = {"anchor1", "anchor2"};
	for (int i=0; i<2; i++)
	{
		CConfigSection anchorNode = configSection.section(anchorNodeNames[i]);
		if (!anchorNode.isNull())
		{
			configresult &= mLogAssert(anchorNode.get("bodyname", &bodyName));
			configresult &= mLogAssert(anchorNode.get("anchorname", &anchorName));
			mpBodyAnchors[i] = mpObject->resolveAnchor(bodyName, anchorName);
			if (mpBodyAnchors[i] == NULL)
				mLogErrorLn("In CODEJoint::readConfig() for object \"" << mpObject->name() << "\": could not resolve anchor \"" << bodyName << "\"->\"" << anchorName << "\"!");
		}
	}

	return configresult;
}

bool CODEJoint::init(dWorld& world, dJointGroupID groupID)
{
	create(world, groupID);
	if (mID != 0)
	{
		if (!assertAnchors())
		{
			mLogErrorLn("CODEJoint::init() failed because one or both anchors were NULL!");
			return false;
		}

		mLogDebugLn("Created " << getTypeName() << "-joint \"" << name() << "\"!");
		// Connect the bodies: move them into place and notify both bodies that they are connected to each other
		connectBodies();
		// Attach bodies to the joint
		dJointAttach(mID, mpBodyAnchors[0]->body()->id(), mpBodyAnchors[1]->body()->id());
		setParams();
	}
	else
		mLogErrorLn("Trying to initialize joint that failed to be created! Check your configuration.");

	return mID != 0;
}

void CODEJoint::connectBodies()
{
	// Displacement calculation = [body1_pos] + [body1_anchorpos] - [body2_anchorpos] - [body2_pos]
	// Displacement calculation = anchor2_pos - anchor1_pos
	double anchorWorldPos[2][3];
	for (int i=0; i<2; i++)
	{
		mpBodyAnchors[i]->getPosition(anchorWorldPos[i]);
		//dbgprintf("[DEBUG] Anchor \"%s\" in body \"%s\" is placed at (%.4f, %.4f, %.4f)\n", mpBodyAnchors[i]->name().c_str(), mpBodyAnchors[i]->body()->name().c_str(), anchorWorldPos[i][0], anchorWorldPos[i][1], anchorWorldPos[i][2]);
	}

	// Move body 2 so that the anchors will overlap
	double dx = anchorWorldPos[0][0] - anchorWorldPos[1][0];
	double dy = anchorWorldPos[0][1] - anchorWorldPos[1][1];
	double dz = anchorWorldPos[0][2] - anchorWorldPos[1][2];
	if (mpBodyAnchors[1]->body()->id() != 0)
	{
		mpBodyAnchors[1]->body()->moveIsland(dx, dy, dz);
		//dbgprintf("[DEBUG] Requested to move body \"%s\" over (%.4f, %.4f, %.4f)\n", mpBodyAnchors[1]->body()->name().c_str(), dx, dy, dz);
	}
	else
	{
		mpBodyAnchors[0]->body()->moveIsland(-dx, -dy, -dz);
		//dbgprintf("[DEBUG] Requested to move body \"%s\" over (%.4f, %.4f, %.4f)\n", mpBodyAnchors[0]->body()->name().c_str(), -dx, -dy, -dz);
	}

	// Add the bodies to each other's connectedbodies list
	for (int i=0; i<2; i++)
		mpBodyAnchors[1-i]->body()->addConnectedBody(mpBodyAnchors[i]->body());
}

void CODEJoint::convertKDToERPCFM(double K, double D, double *ERP, double *CFM)
{
	mpObject->convertKDToERPCFM(K, D, ERP, CFM);
}

CODEHingeJoint::CODEHingeJoint(CODEObject *parentObject):
	CODEJoint(parentObject),
	mMotor(NULL),
	mAxisX(1.0),
	mAxisY(0.0),
	mAxisZ(0.0),
	mStopERP(-1.0),
	mStopCFM(-1.0),
	mDryFriction(0.0),
	mLowerLimit(-dInfinity),
	mUpperLimit(dInfinity)
{
}

CODEHingeJoint::~CODEHingeJoint()
{
	if (mMotor != NULL)
		delete mMotor;
}

bool CODEHingeJoint::init(dWorld& world, dJointGroupID groupID)
{
	bool result = CODEJoint::init(world, groupID);

	if (mDryFriction > 0)
	{
		dJointSetHingeParam(mID, dParamVel, 0);
		dJointSetHingeParam(mID, dParamFMax, mDryFriction);
	}

	return result;
}

void CODEHingeJoint::setMotor(CODEJointMotor* motor)
{
	if (mMotor != NULL)
		delete mMotor;

	mMotor = motor;
}

unsigned int CODEHingeJoint::getNumMotors()
{
	return 1;
}

CODEJointMotor*	CODEHingeJoint::getMotor(int motorIndex)
{
	return mMotor;
}

void CODEHingeJoint::create(dWorld& world, dJointGroupID groupID)
{
	mID = dJointCreateHinge(world, groupID);
}

void CODEHingeJoint::setParams()
{
	double anchorPos[3];
	mpBodyAnchors[0]->getPosition(anchorPos);
	dJointSetHingeAnchor(mID, anchorPos[0], anchorPos[1], anchorPos[2]);
	dJointSetHingeAxis(mID, mAxisX, mAxisY, mAxisZ);
	dJointSetHingeParam(mID, dParamLoStop, mLowerLimit);
	dJointSetHingeParam(mID, dParamHiStop, mUpperLimit);
	if (mStopERP >= 0)
		dJointSetHingeParam(mID, dParamStopERP, mStopERP);
	if (mStopCFM >= 0)
		dJointSetHingeParam(mID, dParamStopCFM, mStopCFM);
}

double CODEHingeJoint::getAngle(int axisIndex) const
{
	return dJointGetHingeAngle(mID);
}
double CODEHingeJoint::getAngleRate(int axisIndex) const
{
	return dJointGetHingeAngleRate(mID);
}

void CODEHingeJoint::addTorque(double torque, int axisIndex)
{
	dJointAddHingeTorque(mID, torque);
}

bool CODEHingeJoint::readConfig(const CConfigSection &configSection)
{
	bool configresult=true;

	configresult &= CODEJoint::readConfig(configSection);

	// Read hinge axis
	configresult &= mLogAssert(configSection.get("axisX", &mAxisX));
	configresult &= mLogAssert(configSection.get("axisY", &mAxisY));
	configresult &= mLogAssert(configSection.get("axisZ", &mAxisZ));
	configSection.get("lowerlimit", &mLowerLimit);
	configSection.get("upperlimit", &mUpperLimit);

	// Handle K-D or ERP-CFM setings
	if (configSection.has("stopK"))
	{
		double K=0, D=0, ERP=0, CFM=0;
		configresult &= mLogAssert(configSection.get("stopK", &K));
		configresult &= mLogAssert(configSection.get("stopD", &D));
		convertKDToERPCFM(K, D, &ERP, &CFM);
		mStopERP = ERP;
		mStopCFM = CFM;
	}
	else
	if (configSection.has("stopERP"))
	{
		configresult &= mLogAssert(configSection.get("stopERP", &mStopERP));
		configresult &= mLogAssert(configSection.get("stopCFM", &mStopCFM));
	}

	// Read static friction
	configSection.get("dryfriction", &mDryFriction);

	// Read motor, if any
	CConfigSection motorSection = configSection.section("motor");
	if (!motorSection.isNull())
	{
		std::string motorTypeStr;
		configresult &= mLogAssert(motorSection.get("type", &motorTypeStr, "undefined"));

		CODEJointMotor *newMotor = gODECreateJointMotor(motorTypeStr, this);

		if (newMotor != NULL)
		{
			configresult &= newMotor->readConfig(motorSection);
			setMotor(newMotor);
		}
	}

	return configresult;
}

CODESliderJoint::CODESliderJoint(CODEObject *parentObject):
	CODEJoint(parentObject),
	mMotor(NULL),
	mAxisX(1.0),
	mAxisY(0.0),
	mAxisZ(0.0),
	mStopERP(-1.0),
	mStopCFM(-1.0),
	mDryFriction(0.0),
	mLowerLimit(-dInfinity),
	mUpperLimit(dInfinity)
{
}

CODESliderJoint::~CODESliderJoint()
{
	if (mMotor != NULL)
		delete mMotor;
}

void CODESliderJoint::create(dWorld& world, dJointGroupID groupID)
{
	mID = dJointCreateSlider(world, groupID);
}

bool CODESliderJoint::init(dWorld& world, dJointGroupID groupID)
{
	bool result = CODEJoint::init(world, groupID);

	if (mDryFriction > 0)
	{
		dJointSetSliderParam(mID, dParamVel, 0);
		dJointSetSliderParam(mID, dParamFMax, mDryFriction);
	}

	return result;
}

void CODESliderJoint::setMotor(CODEJointMotor* motor)
{
	if (mMotor != NULL)
		delete mMotor;

	mMotor = motor;
}

unsigned int CODESliderJoint::getNumMotors()
{
	return 1;
}

CODEJointMotor*	CODESliderJoint::getMotor(int motorIndex)
{
	return mMotor;
}

void CODESliderJoint::setParams()
{
	dJointSetSliderAxis(mID, mAxisX, mAxisY, mAxisZ);
	dJointSetSliderParam(mID, dParamLoStop, mLowerLimit);
	dJointSetSliderParam(mID, dParamHiStop, mUpperLimit);
	if (mStopERP >= 0)
		dJointSetSliderParam(mID, dParamStopERP, mStopERP);
	if (mStopCFM >= 0)
		dJointSetSliderParam(mID, dParamStopCFM, mStopCFM);
}

double CODESliderJoint::getPosition() const
{
	return dJointGetSliderPosition(mID);
}

double CODESliderJoint::getPositionRate() const
{
	return dJointGetSliderPositionRate(mID);
}

void CODESliderJoint::addForce(double force)
{
	dJointAddSliderForce(mID, force);
}

bool CODESliderJoint::readConfig(const CConfigSection &configSection)
{
	bool configresult=true;

	configresult &= CODEJoint::readConfig(configSection);

	// Read hinge axis
	configresult &= mLogAssert(configSection.get("axisX", &mAxisX));
	configresult &= mLogAssert(configSection.get("axisY", &mAxisY));
	configresult &= mLogAssert(configSection.get("axisZ", &mAxisZ));
	//mLogDebugLn("SliderJoint axisX: "<< mAxisX <<", axisY: "<< mAxisY <<", axisZ: "<< mAxisZ);
	configSection.get("lowerlimit", &mLowerLimit);
	configSection.get("upperlimit", &mUpperLimit);

	// Handle K-D or ERP-CFM setings
	if (configSection.has("stopK"))
	{
		double K=0, D=0, ERP=0, CFM=0;
		configresult &= mLogAssert(configSection.get("stopK", &K));
		configresult &= mLogAssert(configSection.get("stopD", &D));
		convertKDToERPCFM(K, D, &ERP, &CFM);
		mStopERP = ERP;
		mStopCFM = CFM;
	}
	else
	if (configSection.has("stopERP"))
	{
		configresult &= mLogAssert(configSection.get("stopERP", &mStopERP));
		configresult &= mLogAssert(configSection.get("stopCFM", &mStopCFM));
	}

	// Read static friction
	configSection.get("dryfriction", &mDryFriction);

	// Read motor, if any
	CConfigSection motorSection = configSection.section("motor");
	if (!motorSection.isNull())
	{
		std::string motorTypeStr;
		configresult &= mLogAssert(motorSection.get("type", &motorTypeStr, "undefined"));

		CODEJointMotor *newMotor = gODECreateJointMotor(motorTypeStr, this);

		if (newMotor != NULL)
		{
			configresult &= newMotor->readConfig(motorSection);
			setMotor(newMotor);
		}
	}

	return configresult;
}

void CODEUniversalJoint::create(dWorld& world, dJointGroupID groupID)
{
	mID = dJointCreateUniversal(world, groupID);
}

void CODEUniversalJoint::setParams()
{
	double anchorPos[3];
	mpBodyAnchors[0]->getPosition(anchorPos);
	dJointSetUniversalAnchor(mID, anchorPos[0], anchorPos[1], anchorPos[2]);
	dJointSetUniversalAxis1(mID, mAxis1X, mAxis1Y, mAxis1Z);
	dJointSetUniversalAxis1(mID, mAxis2X, mAxis2Y, mAxis2Z);
}

bool CODEUniversalJoint::readConfig(const CConfigSection &configSection)
{
	bool configresult=true;

	configresult &= CODEJoint::readConfig(configSection);

	// Read hinge axes
	configresult &= mLogAssert(configSection.get("axis1X", &mAxis1X));
	configresult &= mLogAssert(configSection.get("axis1Y", &mAxis1Y));
	configresult &= mLogAssert(configSection.get("axis1Z", &mAxis1Z));
	configresult &= mLogAssert(configSection.get("axis2X", &mAxis2X));
	configresult &= mLogAssert(configSection.get("axis2Y", &mAxis2Y));
	configresult &= mLogAssert(configSection.get("axis2Z", &mAxis2Z));
	mLogDebugLn("UniversalJoint axis1X: "<< mAxis1X <<", axis1Y: "<< mAxis1Y <<", axis1Z: "<< mAxis1Z);
	mLogDebugLn("UniversalJoint axis2X: "<< mAxis2X <<", axis2Y: "<< mAxis2Y <<", axis2Z: "<< mAxis2Z);

	return configresult;
}

void CODEBallJoint::create(dWorld& world, dJointGroupID groupID)
{
	mID = dJointCreateBall(world, groupID);
}

void CODEBallJoint::setParams()
{
	double anchorPos[3];
	mpBodyAnchors[0]->getPosition(anchorPos);
	dJointSetBallAnchor(mID, anchorPos[0], anchorPos[1], anchorPos[2]);
}

void CODEFixedJoint::create(dWorld& world, dJointGroupID groupID)
{
	mID = dJointCreateFixed(world, groupID);
}

void CODEFixedJoint::setParams()
{
	dJointSetFixed(mID);
}

// ***************************** //
// Global class factory function //
// ***************************** //

CODEJoint* gODECreateJoint(const std::string& jointTypeStr, CODEObject *parentObject)
{
	CODEJoint* newJoint = NULL;

	if (jointTypeStr == "ball")
		newJoint = new CODEBallJoint(parentObject);
	else if (jointTypeStr == "hinge")
		newJoint = new CODEHingeJoint(parentObject);
	else if (jointTypeStr == "slider")
		newJoint = new CODESliderJoint(parentObject);
	else if (jointTypeStr == "universal")
		newJoint = new CODEUniversalJoint(parentObject);
	else if (jointTypeStr == "fixed")
		newJoint = new CODEFixedJoint(parentObject);

	return newJoint;
}
