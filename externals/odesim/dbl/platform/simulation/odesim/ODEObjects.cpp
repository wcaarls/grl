/*
 *    Implementation of CGround, CGoal, CBall and CBallState.
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

#include "ODEObjects.h"
#include "ODESim.h"

mu::Parser CODEInitializationExpression::mParser;
grl::Rand CODEInitializationExpression::mRand;

CODEObjectFixedPoint::CODEObjectFixedPoint()
{
	mBodyX = mBodyY = mBodyZ = 0;
	mWorldX = mWorldY = mWorldZ = 0;
}

bool CODEObjectFixedPoint::readConfig(const CConfigSection &configSection)
{
	bool configresult=true;
	std::string worldX, worldY, worldZ;

	configresult &= mLogAssert(configSection.get("bodyname", &mBodyName, "bodyname-missing"));

	configresult &= mLogAssert(configSection.get("bodyX",  &mBodyX));
	configresult &= mLogAssert(configSection.get("bodyY",  &mBodyY));
	configresult &= mLogAssert(configSection.get("bodyZ",  &mBodyZ));

	configresult &= mLogAssert(configSection.get("worldX", &worldX));
	configresult &= mLogAssert(configSection.get("worldY", &worldY));
	configresult &= mLogAssert(configSection.get("worldZ", &worldZ));

	mWorldXExpr.setExpression(worldX);
	mWorldYExpr.setExpression(worldY);
	mWorldZExpr.setExpression(worldZ);

	randomize();

	return configresult;
}

void CODEObjectFixedPoint::randomize()
{
  mWorldX = mWorldXExpr.evaluate();
  mWorldY = mWorldYExpr.evaluate();
  mWorldZ = mWorldZExpr.evaluate();
}

CODEBodyIC::CODEBodyIC()
{

}

bool CODEBodyIC::readConfig(const CConfigSection &configSection)
{
	bool configresult=true;

	configresult &= mLogAssert(configSection.get("bodyname", &mBodyName, "bodyname-missing"));

  CConfigSection orientationSection = configSection.section("orientation");
  CConfigSection rotationSection    = configSection.section("rotation");

  if (!orientationSection.isNull())
  {
    std::string ax, ay, az, bx, by, bz;

    mOrientationAsRotation = false;
    CConfigSection xAxisSection = orientationSection.section("Xaxis");
    configresult &= mLogAssert(xAxisSection.get("x", &ax));
    configresult &= mLogAssert(xAxisSection.get("y", &ay));
    configresult &= mLogAssert(xAxisSection.get("z", &az));
    CConfigSection yAxisSection = orientationSection.section("Yaxis");
    configresult &= mLogAssert(yAxisSection.get("x", &bx));
    configresult &= mLogAssert(yAxisSection.get("y", &by));
    configresult &= mLogAssert(yAxisSection.get("z", &bz));

    mXaxisX.setExpression(ax);
    mXaxisY.setExpression(ay);
    mXaxisZ.setExpression(az);
    mYaxisX.setExpression(bx);
    mYaxisY.setExpression(by);
    mYaxisZ.setExpression(bz);
  }
  else if (!rotationSection.isNull())
    // Try reading a rotation
  {
    std::string ax, ay, az, angle;

    mOrientationAsRotation = true;
    CConfigSection axisSection = rotationSection.section("axis");
    configresult &= mLogAssert(axisSection.get("x", &ax));
    configresult &= mLogAssert(axisSection.get("y", &ay));
    configresult &= mLogAssert(axisSection.get("z", &az));
    configresult &= mLogAssert(rotationSection.get("angle", &angle));

    mAxisX.setExpression(ax);
    mAxisY.setExpression(ay);
    mAxisZ.setExpression(az);
    mAngle.setExpression(angle);
  }

  randomize();

	return configresult;
}

void CODEBodyIC::randomize(double r)
{
  if (mOrientationAsRotation)
    mRotation.setRotation(mAxisX.evaluate(), mAxisY.evaluate(), mAxisZ.evaluate(), mAngle.evaluate() + r);
  else
    mRotation.setOrientation(mXaxisX.evaluate(), mXaxisY.evaluate(), mXaxisZ.evaluate(),
                             mYaxisX.evaluate(), mYaxisY.evaluate(), mYaxisZ.evaluate());
}

CODEExtForce::CODEExtForce(CODEObject *parentObject):
	mpObject(parentObject),
	mForceX(0.0),
	mForceY(0.0),
	mForceZ(0.0),
	mBodyX(0.0),
	mBodyY(0.0),
	mBodyZ(0.0)
{
}

bool CODEExtForce::init()
{
	mpBody = mpObject->resolveBody(mBodyName);
	// Fail?
	if (mpBody == NULL)
	{
		mLogErrorLn("In CODEExtForce::init(): bodyname \"" << mBodyName << "\" not found in object \"" << mpObject->name() << "\"!");
		return false;
	}
	else
		return true;
}

void CODEExtForce::update()
{
	// Force vector in GLOBAL reference frame, point on body in BODY's reference frame --> dBodyAddForceAtRelPos()
	if (mpBody != NULL)
		mpBody->addForceAtRelPos(mForceX, mForceY, mForceZ, mBodyX, mBodyY, mBodyZ);
}

bool CODEExtForce::readConfig(const CConfigSection &configSection)
{
	bool configresult=true;

	configresult &= mLogAssert(configSection.get("bodyname", &mBodyName, "bodyname-missing"));

	configresult &= mLogAssert(configSection.get("forceX",  &mForceX));
	configresult &= mLogAssert(configSection.get("forceY",  &mForceY));
	configresult &= mLogAssert(configSection.get("forceZ",  &mForceZ));

	configresult &= mLogAssert(configSection.get("bodyX",  &mBodyX));
	configresult &= mLogAssert(configSection.get("bodyY",  &mBodyY));
	configresult &= mLogAssert(configSection.get("bodyZ",  &mBodyZ));

	return configresult;
}

CODEObject::CODEObject(CODESim* pSim):
	mpSim(pSim)
{
	mFixedPoint			= NULL;
	mShouldDrawCoMs		= false;
	mShouldDrawBodies	= true;
	mShouldDrawGeoms	= false;
	// Disable cleanup mode of the hash space - we will delete the geoms one by one in the destructors.
	mSpace.setCleanup(0);
  mRand.init(time(NULL));
}

CODEObject::~CODEObject()
{
	if (mFixedPoint != NULL)
		delete mFixedPoint;
}

const std::string& CODEObject::name() const
{
	return mName;
}

void CODEObject::drawCoMs(bool enabled)
{
	mShouldDrawCoMs = enabled;
}

bool CODEObject::shouldDrawCoMs()
{
	return mShouldDrawCoMs;
}

void CODEObject::drawBodies(bool enabled)
{
	mShouldDrawBodies = enabled;
}

bool CODEObject::shouldDrawBodies()
{
	return mShouldDrawBodies;
}

void CODEObject::drawJoints(bool enabled)
{
	mShouldDrawJoints = enabled;
}

bool CODEObject::shouldDrawJoints()
{
	return mShouldDrawJoints;
}

void CODEObject::drawGeoms(bool enabled)
{
	mShouldDrawGeoms = enabled;
}

bool CODEObject::shouldDrawGeoms()
{
	return mShouldDrawGeoms;
}

bool CODEObject::init(dWorld& world)
{
	// Init bodies
	for (unsigned int iBody=0; iBody<mBodies.size(); iBody++)
	{
		mBodies[iBody]->init(world);
	}
	//dbgprintf("[DEBUG] Object \"%s\": bodies initialized\n", name().c_str());

	// Init geometries
	for (unsigned int iGeom=0; iGeom<mGeoms.size(); iGeom++)
	{
		mGeoms[iGeom]->init(mSpace);
	}
	//dbgprintf("[DEBUG] Object \"%s\": geoms initialized\n", name().c_str());

	// Init joints
	for (unsigned int iJoint=0; iJoint<mJoints.size(); iJoint++)
	{
		mJoints[iJoint]->init(world);
	}
	//dbgprintf("[DEBUG] Object \"%s\": joints initialized\n", name().c_str());

	// DEBUG: report joint connections:
	/*
	for (unsigned int iBody=0; iBody<mBodies.size(); iBody++)
	{
		dbgprintf("[DEBUG] Body \"%s\" is connected to: ", mBodies[iBody]->name().c_str());
		for (unsigned int iConBody=0; iConBody<mBodies[iBody]->getConnectedBodies().size(); iConBody++)
		{
			dbgprintf("\"%s\", ", mBodies[iBody]->getConnectedBodies()[iConBody]->name().c_str());
		}
		dbgprintf("\n");
	}
	*/

	setInitialCondition(false);

	// Init external forces
	for (unsigned int iForce=0; iForce<mExternalForces.size(); iForce++)
		mExternalForces[iForce]->init();

	return true;	// Do we want to return something else? Can we ever fail?
}

void CODEObject::genRandState(std::map<std::string, double> &jointMap)
{
  const double C = 1*0.087263889; // 0.087263889 = +/- 5 deg
  double r1 = mRand.getUniform(-C, C);
  double r2 = mRand.getUniform(0, 2*C); // knee cannot be bended outside
  double r3 = mRand.getUniform(-C, C);

  jointMap[std::string("virtualBoom")] = 0;
  jointMap[std::string("torso")] = r3;
  jointMap[std::string("upperlegleft")] = r1;
  jointMap[std::string("upperlegright")] = r1;
  jointMap[std::string("lowerlegleft")] = r2;
  jointMap[std::string("lowerlegright")] = r2;
  jointMap[std::string("footleft")] = 0;
  jointMap[std::string("footright")] = 0;
  jointMap[std::string("arm")] = r3;
}

void CODEObject::setInitialCondition(bool randomize)
{
  std::map<std::string, double> jointMap;
  if (randomize)
    genRandState(jointMap);

	// Process body ICs
	for (unsigned int iBodyIC=0; iBodyIC<mBodyICs.size(); iBodyIC++)
	{
		CODEBody *body = resolveBody(mBodyICs[iBodyIC]->getBodyName());
		// Fail?
		if (body == NULL)
		{
			mLogErrorLn("In CODEObject::setInitialCondition(): could not resolve body " << mBodyICs[iBodyIC]->getBodyName() <<"!");
		}
		else
		{
			if (randomize)
        mBodyICs[iBodyIC]->randomize(jointMap[mBodyICs[iBodyIC]->getBodyName()]);
			body->setRotation(mBodyICs[iBodyIC]->getRotation());
		}
	}

	// Set velocities to their initial condition
	// Since we currently do not support custom velocities,
	// we set all velocities (angular and linear) to zero.
	for (unsigned int iBody=0; iBody<mBodies.size(); iBody++)
		mBodies[iBody]->zero();

	// Now that body rotations have moved, first disconnect all bodies ...
	for (unsigned int iBody=0; iBody<mBodies.size(); iBody++)
		mBodies[iBody]->disconnectBodies();
	// ... and then assemble the object again
	for (unsigned int iJoint=0; iJoint<mJoints.size(); iJoint++)
		mJoints[iJoint]->connectBodies();

	// Process fixed point after initializing the joints,
	// due to the possibility that joints could be created
	// with anchor points in the fixed world
	if (mFixedPoint)
	{
		if (randomize)
			mFixedPoint->randomize();
		processFixedPoint();
	}

	// Update the internal state of the motors according to the new initial state (pos and vel) of the connected bodies
	for (unsigned int iJoint=0; iJoint<mJoints.size(); iJoint++)
		for (unsigned int iMotor=0; iMotor<mJoints[iJoint]->getNumMotors(); iMotor++)
		{
			CODEJointMotor* motor = mJoints[iJoint]->getMotor(iMotor);
			if (motor != NULL)
				motor->setInitialCondition();
		}
}

void CODEObject::deinit()
{
	clearAll();
}

void CODEObject::addBody(CODEBody* pBody)
{
	mBodies.push_back(pBody);
}

void CODEObject::addGeom(CODEGeom* pGeom)
{
	mGeoms.push_back(pGeom);
}

void CODEObject::addJoint(CODEJoint* pJoint)
{
	mJoints.push_back(pJoint);
}

void CODEObject::addBodyIC(CODEBodyIC* pBodyIC)
{
	mBodyICs.push_back(pBodyIC);
}

void CODEObject::addExternalForce(CODEExtForce* pExtForce)
{
	mExternalForces.push_back(pExtForce);
}

void CODEObject::clearBodies()
{
	for (unsigned int iBody=0; iBody<mBodies.size(); iBody++)
	{
		mBodies[iBody]->deinit();
		delete mBodies[iBody];
	}
	mBodies.clear();
}

void CODEObject::clearGeoms()
{
	for (unsigned int iGeom=0; iGeom<mGeoms.size(); iGeom++)
	{
		mGeoms[iGeom]->deinit();
		delete mGeoms[iGeom];
	}
	mGeoms.clear();
}

void CODEObject::clearJoints()
{
	for (unsigned int iJoint=0; iJoint<mJoints.size(); iJoint++)
	{
		mJoints[iJoint]->deinit();
		delete mJoints[iJoint];
	}
	mJoints.clear();
}

void CODEObject::clearBodyICs()
{
	for (unsigned int iBodyIC=0; iBodyIC<mBodyICs.size(); iBodyIC++)
		delete mBodyICs[iBodyIC];

	mBodyICs.clear();
}

void CODEObject::clearExternalForces()
{
	for (unsigned int iForce=0; iForce<mExternalForces.size(); iForce++)
		delete mExternalForces[iForce];

	mExternalForces.clear();
}

void CODEObject::clearAll()
{
	// Deinit bodies
	clearBodies();

	// Deinit geometries
	clearGeoms();

	// Deinit joints
	clearJoints();

	// Deinit body ICs
	clearBodyICs();

	// Deinit external forces
	clearExternalForces();
}

void CODEObject::getCOM(double &x, double &y, double &z) const
{
  x = y = z = 0;
  double M = 0, m;
  for (unsigned int i=0; i<mBodies.size(); i++)
  {
    m = mBodies[i]->getMass();
    const dReal *pos = ((dBody*)mBodies[i])->getPosition();
    x += pos[0]*m;
    y += pos[1]*m;
    z += pos[2]*m;
    M += m;
  }
  x = x/M;
  y = y/M;
  z = z/M;
}

void CODEObject::move(double dx, double dy, double dz)
{
	for (unsigned int i=0; i<mBodies.size(); i++)
		mBodies[i]->move(dx, dy, dz);
}

void CODEObject::invalidateCollisions(bool invalid)
{
	mpSim->invalidateCollisions(invalid);
}

void CODEObject::processFixedPoint()
{
	if (mFixedPoint == NULL)
		return;

	CODEBody *fixedBody = resolveBody(mFixedPoint->getBodyName());
	// Fail?
  if (fixedBody == NULL)
	{
		mLogErrorLn("In CODEObject::processFixedPoint(): could not resolve body " << mFixedPoint->getBodyName() << "!");
		return;
	}

	// Now move!
	double fixedPointPos[3];
	fixedBody->getPosition(fixedPointPos, mFixedPoint->getBodyX(), mFixedPoint->getBodyY(), mFixedPoint->getBodyZ());
	move(mFixedPoint->getWorldX() - fixedPointPos[0], mFixedPoint->getWorldY() - fixedPointPos[1], mFixedPoint->getWorldZ() - fixedPointPos[2]);
}

bool CODEObject::readConfig(const CConfigSection &configSection)
{
	bool configresult=true, result;

	// Clear everything inside the object before loading a new configuration
	clearAll();

	configresult &= mLogAssert(configSection.get("name", &mName, "nameless-object"));
	configSection.get("drawbodies", &mShouldDrawBodies);
	configSection.get("drawjoints", &mShouldDrawJoints);
	configSection.get("drawcoms", &mShouldDrawCoMs);
	configSection.get("drawgeoms", &mShouldDrawGeoms);


	// Read bodies
	for (CConfigSection bodyNode = configSection.section("body"); !bodyNode.isNull(); bodyNode = bodyNode.nextSimilarSection())
	{
		CODEBody *newBody = new CODEBody(this);
		configresult &= result = newBody->readConfig(bodyNode);
		if (!result)
			mLogErrorLn("CODEObject::readConfig() failed to read body " << newBody->name() << "!");
		addBody(newBody);
	}

	// Read geoms
	for (CConfigSection geomNode = configSection.section("geom"); !geomNode.isNull(); geomNode = geomNode.nextSimilarSection())
	{
		std::string geomTypeStr;
		configresult &= mLogAssert(geomNode.get("type", &geomTypeStr, "undefined"));

		CODEGeom *newGeom = gODECreateGeom(geomTypeStr, this);

		if (newGeom != NULL)
		{
			configresult &= result = newGeom->readConfig(geomNode);
			if (!result)
				mLogErrorLn("CODEObject::readConfig() failed to read geom " << newGeom->name() << "!");
			addGeom(newGeom);
		}
	}

	// Read joints
	for (CConfigSection jointNode = configSection.section("joint"); !jointNode.isNull(); jointNode = jointNode.nextSimilarSection())
	{
		std::string jointTypeStr;
		configresult &= mLogAssert(jointNode.get("type", &jointTypeStr, "undefined"));

		CODEJoint *newJoint = gODECreateJoint(jointTypeStr, this);

		if (newJoint != NULL)
		{
			configresult &= result = newJoint->readConfig(jointNode);
			if (!result)
				mLogErrorLn("CODEObject::readConfig() failed to read joint " << newJoint->name() << "!");
			addJoint(newJoint);
		}
	}

	// Read boody initial conditions
	for (CConfigSection ICNode = configSection.section("initialcondition"); !ICNode.isNull(); ICNode = ICNode.nextSimilarSection())
	{
		CODEBodyIC *newBodyIC = new CODEBodyIC;
		if (!newBodyIC->readConfig(ICNode))
		{
			mLogErrorLn("CODEObject::readConfig() failed to read initial condition for body " << newBodyIC->getBodyName() << "!");
			configresult = false;
		}
		else
			addBodyIC(newBodyIC);
	}

	// Read external forces
	for (CConfigSection forceNode = configSection.section("externalforce"); !forceNode.isNull(); forceNode = forceNode.nextSimilarSection())
	{
		CODEExtForce *newForce = new CODEExtForce(this);
		if (!newForce->readConfig(forceNode))
		{
			mLogErrorLn("CODEObject::readConfig() failed to read external force for body " << newForce->getBodyName() << "!");
			configresult = false;
		}
		else
			addExternalForce(newForce);
	}

	// Read fixed pos, if any
	CConfigSection fixedPointNode = configSection.section("fixedpoint");
	if (!fixedPointNode.isNull())
	{
		if (mFixedPoint != NULL)
			delete mFixedPoint;
		mFixedPoint = new CODEObjectFixedPoint();
		configresult &= mFixedPoint->readConfig(fixedPointNode);
	}

	return configresult;
}

CODEBody* CODEObject::resolveBody(const std::string &bodyName)
{
	CODEBody* foundBody = NULL;
	if (bodyName == "world")
	{
		foundBody = mpSim->getWorldBody();
		//dbgprintf("[DEBUG] Resolved anchor to the fixed world (mpWorldBody %s)\n", (mpWorldBody==NULL)?"is NULL!":"is not NULL");
	}
	else
	{
		for (unsigned int iBody=0; iBody<mBodies.size(); iBody++)
		{
			if (mBodies[iBody]->name() == bodyName)
			{
				foundBody = mBodies[iBody];
				break;
			}
		}
	}
	return foundBody;
}

CODEJoint* CODEObject::resolveJoint(const std::string &jointName)
{
	CODEJoint* foundJoint = NULL;
	for (unsigned int iJoint=0; iJoint<mJoints.size(); iJoint++)
	{
		if (mJoints[iJoint]->name() == jointName)
		{
			foundJoint = mJoints[iJoint];
			break;
		}
	}
	return foundJoint;
}

CODEGeom* CODEObject::resolveGeom(const std::string &geomName)
{
	CODEGeom* foundGeom = NULL;
	for (unsigned int iGeom=0; iGeom<mGeoms.size(); iGeom++)
	{
		if (mGeoms[iGeom]->name() == geomName)
		{
			foundGeom = mGeoms[iGeom];
			break;
		}
	}
	return foundGeom;
}

void CODEObject::convertKDToERPCFM(double K, double D, double *ERP, double *CFM)
{
	mpSim->convertKDToERPCFM(K, D, ERP, CFM);
}

CODEBodyAnchor* CODEObject::resolveAnchor(const std::string &bodyName, const std::string &anchorName)
{
	CODEBody* foundBody = resolveBody(bodyName);

	CODEBodyAnchor* foundAnchor = NULL;
	if (foundBody != NULL)
	{
		for (unsigned int iAnchor=0; iAnchor<foundBody->getAnchors().size(); iAnchor++)
		{
			if (foundBody->getAnchors()[iAnchor]->name() == anchorName)
			{
				foundAnchor = foundBody->getAnchors()[iAnchor];
				break;
			}
		}
	}

	return foundAnchor;
}

//******* CGround **************************

/*

CGround::CGround() : CODEGeom()
{
	mA = 0;
	mB = 0;
	mC = 1;
	mD = 0;

	mBeams = NULL;
	mNumBeams = 0;
	mDrawGrid = false;
	mDrawXY = false;//true;
}

CGround::~CGround()
{
	CleanupBeams();
}

void CGround::DrawGrid(const bool enable)
{
	mDrawGrid = enable;
}

void CGround::CleanupBeams()
{
	// Save oldNumBeams and directly set mNumBeams to 0 because otherwise Draw() will draw already deleted beams.
	int oldNumBeams = mNumBeams;
	mNumBeams = 0;
	if (oldNumBeams > 0)
	{
		for (int i=0; i<oldNumBeams; i++)
			dGeomDestroy(mBeams[i]);
		delete[] mBeams;
		mBeams = NULL;
	}
}

// Init and Deinit (mostly operate in the ODE world)
void CGround::Init(dSpaceID spaceID)
{
	mGeomID	= dCreatePlane(spaceID, mA, mB, mC, mD);
	mSpaceID = spaceID;
}

void CGround::Deinit()
{
	dGeomDestroy(mGeomID);
	CleanupBeams();
}

void CGround::SetPlaneParams(dReal a, dReal b, dReal c, dReal d)
{
	mA = a;
	mB = b;
	mC = c;
	mD = d;

	if (mGeomID != NULL)
		dGeomPlaneSetParams(mGeomID, a, b, c, d);
}

void CGround::SetZRamp(const dReal rampRadians)
{
	SetPlaneParams(-sin(rampRadians), 0, cos(rampRadians), 0);
}

void CGround::Draw()
{
	if (mDrawGrid)
	{
		dsSetColor(0.2f, 0.2f, 0.2f);
		const int numLinesX = 40;
		const int numLinesY = 10;
		dReal yMin = -1, yMax = 1, xMin = -10, xMax = 10;
		dReal pos1[3] = {0, yMin, 0};
		dReal pos2[3] = {0, yMax, 0};
		for (int x = 0; x<numLinesX; x++)
		{
			pos1[0] = xMin + x*(xMax - xMin)/((float)numLinesX-1.0);
			pos1[2] = (mD - mB*yMin - mA*pos1[0]) / mC;
			pos2[0] = pos1[0];
			pos2[2] = (mD - mB*yMax - mA*pos2[0]) / mC;
			dsDrawLine(pos1, pos2);
		}

		dReal pos3[3] = {xMin, 0, 0};
		dReal pos4[3] = {xMax, 0, 0};
		for (int y=0; y<numLinesY; y++)
		{
			pos3[1] = yMin + y*(yMax - yMin)/((float)numLinesY-1.0);
			pos3[2] = (mD - mA*xMin - mB*pos3[1]) / mC;
			pos4[1] = pos3[1];
			pos4[2] = (mD - mA*xMax - mB*pos4[1]) / mC;
			dsDrawLine(pos3, pos4);
		}
	}

	if (mDrawXY)
	{
		dReal xAxisStart[3] = {0, 0, 0.01f};
		dReal xAxisEnd[3] = {1, 0, 0.01f};
		dReal xArrow1[3] = { 0.9f, -0.1f, 0.01f};
		dReal xArrow2[3] = { 0.9f, 0.1f, 0.01f};
		dReal yAxisStart[3] = {0,0,0.01f};
		dReal yAxisEnd[3] = {0,1,0.01f};
		dReal yArrow1[3] = { -0.1f, 0.9f, 0.01f};
		dReal yArrow2[3] = { 0.1f, 0.9f, 0.01f};

		dsSetColor(1,0,0);
		dsDrawLine(xAxisStart, xAxisEnd);
		dsDrawLine(xArrow1, xAxisEnd);
		dsDrawLine(xArrow2, xAxisEnd);
		dsSetColor(0,0,1);
		dsDrawLine(yAxisStart, yAxisEnd);
		dsDrawLine(yArrow1, yAxisEnd);
		dsDrawLine(yArrow2, yAxisEnd);
	}


	// Draw beam boxes
	dMatrix3 I;
	dRSetIdentity(I);
	dVector3 sides;
	dsSetColor(0.6f, 0.6f, 0);
	for (int i=0; i<mNumBeams; i++)
	{
		dGeomBoxGetLengths(mBeams[i], sides);
		dsDrawBox(dGeomGetPosition(mBeams[i]), I, sides);
	}
}

void CGround::CreateBeam(dReal xLength, dReal yLength, dReal zLength, dReal xPos, dReal yPos, dReal zPos)
{

	// Cleanup old beams
	CleanupBeams();


	int count = 1;
	mBeams = new dGeomID[count];

	// Create beam
	mBeams[0] = dCreateBox(mSpaceID, xLength, yLength, zLength);
	dGeomSetPosition(mBeams[0], xPos, yPos, zPos);
	mNumBeams = 1;
}

void CGround::CreateRandomYBeams(dReal xLength, dReal yLength, dReal xPosMin, dReal xPosMax, dReal heightMin, dReal heightMax)
{

	// Cleanup old beams
	CleanupBeams();

	int count = xPosMax/xLength;
	mBeams = new dGeomID[count];

	// Create beams
	dReal zOffset = 0.00001;
	srand( (unsigned)clock() );
	for (int i=0; i<count; i++)
	{
		dReal xPos		= xPosMin + (i + 0.5)*xLength;
		dReal zLength	= -zOffset + heightMin + (heightMax - heightMin) * (float)rand()/(float)RAND_MAX;
		mBeams[i] = dCreateBox(mSpaceID, xLength, yLength, zLength);
		dGeomSetPosition(mBeams[i], xPos, 0, zOffset + zLength*0.5);
	}

	mNumBeams = count;
}

*/
