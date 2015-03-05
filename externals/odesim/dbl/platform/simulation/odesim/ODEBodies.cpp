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

#include "ODEBodies.h"
#include "ODEObjects.h"

bool CODEBodyAnchor::readConfig(const CConfigSection &configSection)
{
	bool configresult=true;

	configresult &= mLogAssert(configSection.get("name", &mName, "nameless-anchor"));
	configresult &= mLogAssert(configSection.get("x",  &mX, 0.0));
	configresult &= mLogAssert(configSection.get("y",  &mY, 0.0));
	configresult &= mLogAssert(configSection.get("z",  &mZ, 0.0));

	return configresult;
}


CODEBody::CODEBody(CODEObject* pParentObject):
	mpObject(pParentObject),
	mName("nameless-body")
{
	mMass				= 1.0;
	mIXX = mIYY = mIZZ	= 1.0;
	mIXY = mIXZ = mIYZ	= 0.0;
	mCMX = mCMY = mCMZ	= 0.0;
}

CODEBody::~CODEBody()
{
	clearAnchors();
	clearDrawingObjects();
}

void CODEBody::clearAnchors()
{
	for (unsigned int i=0; i<mAnchors.size(); i++)
		delete mAnchors[i];

	mAnchors.clear();
}

void CODEBody::clearDrawingObjects()
{
	for (unsigned int i=0; i<mDrawingObjects.size(); i++)
		delete mDrawingObjects[i];

	mDrawingObjects.clear();
}

void CODEBody::setMass()
{
	dMass m;
	m.setParameters(mMass, 0,0,0, mIXX, mIYY, mIZZ, mIXY, mIXZ, mIYZ);
	dBody::setMass(&m);
}

void CODEBody::init(dWorld& world)
{
	create(world);
	setData(this);
	setMass();
	mLogDebugLn("Created CODEBody \"" << mName << "\"!");
	mLogCrawlLn("Mass " << mMass << ", IXX " << mIXX << ", IYY " << mIYY << ", IZZ " << mIZZ);
}

void CODEBody::deinit()
{
	// Destroy is done in the destructor
}

void CODEBody::zero()
{
	dVector3 pos = {0, 0, 0};
	dBody::setPosition(pos);
	dBody::setAngularVel(0, 0, 0);
	dVector3 nullVec = {0, 0, 0, 0};
	dBody::setLinearVel(nullVec);
	mpObject->invalidateCollisions();
}

void CODEBody::move(double dx, double dy, double dz)
{
	// Use dBody::getPosition to request the center of mass position.
	// dx, dy and dz are relative anyway and we don't want to get confused with the center of mass offset.
	const dReal *pos = dBody::getPosition();

	dVector3 finalPos;
	// Add displacement to the body's pos. Because dx, dy and dz are in the GLOBAL reference frame,
	// do NOT use getRelPointPos here
	finalPos[0] = pos[0] + dx;
	finalPos[1] = pos[1] + dy;
	finalPos[2] = pos[2] + dz;

	// Move body's center of mass to new global coordinates
	dBody::setPosition(finalPos);
	mpObject->invalidateCollisions();
}

void CODEBody::moveIsland(double dx, double dy, double dz)
{
	// First, build a vector with the bodies in the island ( = set of interconnected bodies)
	CODEBodyIsland island;
	island.buildIsland(this);
	island.move(dx, dy, dz);
}

void CODEBody::addAnchor(CODEBodyAnchor* newAnchor)
{
	mAnchors.push_back(newAnchor);
}

void CODEBody::addAnchor(std::string name, double x, double y, double z)
{
	CODEBodyAnchor *newAnchor = new CODEBodyAnchor(this, name, x, y, z);
	addAnchor(newAnchor);
}

void CODEBody::addDrawingObject(CSimVisObject* newObject)
{
	mDrawingObjects.push_back(newObject);
}

CSimVisObject* CODEBody::resolveDrawingObject(const std::string &drawingObjectName)
{
	CSimVisObject* result = NULL;
	for (unsigned int iDO=0; iDO<mDrawingObjects.size(); iDO++)
	{
		if (mDrawingObjects[iDO]->name() == drawingObjectName)
		{
			result = mDrawingObjects[iDO];
			break;
		}
	}
	return result;
}

const std::string& CODEBody::name() const
{
	return mName;
}

bool CODEBody::readAnchorsConfig(const CConfigSection &bodyConfigSection)
{
	bool configresult=true;
	clearAnchors();
	for (CConfigSection anchorNode = bodyConfigSection.section("anchor"); !anchorNode.isNull(); anchorNode = anchorNode.nextSimilarSection())
	{
		CODEBodyAnchor *newAnchor = new CODEBodyAnchor(this);
		configresult &= newAnchor->readConfig(anchorNode);
		addAnchor(newAnchor);
	}
	return configresult;
}

bool CODEBody::readDrawingObjectsConfig(const CConfigSection &bodyConfigSection)
{
	bool configresult=true;
	clearDrawingObjects();
	// Get drawinfo node
	CConfigSection drawinfoNode = bodyConfigSection.section("drawinfo");
	if (!drawinfoNode.isNull())
	{
		// Read drawing objects
		for (CConfigSection drawingObjectNode = drawinfoNode.firstSection(); !drawingObjectNode.isNull(); drawingObjectNode = drawingObjectNode.nextSection())
		{
			std::string objectTypeStr = drawingObjectNode.name();
			CSimVisObject *newObject = gCreateSimVisObject(objectTypeStr, this);

			if (newObject != NULL)
			{
				configresult &= newObject->readConfig(drawingObjectNode);
				addDrawingObject(newObject);
			}
		}
	}
	return configresult;
}

bool CODEBody::readConfig(const CConfigSection &configSection)
{
	bool configresult=true;

	configresult &= mLogAssert(configSection.get("name", &mName));
	configresult &= mLogAssert(configSection.get("mass", &mMass));
	configresult &= mLogAssert(configSection.get("IXX",  &mIXX));
	configresult &= mLogAssert(configSection.get("IYY",  &mIYY));
	configresult &= mLogAssert(configSection.get("IZZ",  &mIZZ));
	// It doesn't harm to miss the off-diagonal inertia elements
	configSection.get("IXY",  &mIXY);
	configSection.get("IXZ",  &mIXZ);
	configSection.get("IYZ",  &mIYZ);

	// It doesn't harm not to specify an offset for the center of mass
	configSection.get("CMX",  &mCMX);
	configSection.get("CMY",  &mCMY);
	configSection.get("CMZ",  &mCMZ);

	// Read anchors
	readAnchorsConfig(configSection);

	// Read drawing objects
	readDrawingObjectsConfig(configSection);

	return configresult;
}

void CODEBody::setRotation(const dMatrix3 R)
{
	dBody::setRotation(R);
	mpObject->invalidateCollisions();
}

void CODEBody::addForceAtRelPos(double forceX, double forceY, double forceZ, double posX, double posY, double posZ)
{
	dBody::addForceAtRelPos(forceX, forceY, forceZ, posX - mCMX, posY - mCMY, posZ - mCMZ);
}

void CODEBodyIsland::buildIsland(CODEBody* body)
{
	mBodies.clear();
	addAndProcessBody(body);	// Recursively adds attached bodies too
}

void CODEBodyIsland::addAndProcessBody(CODEBody* body)
{
	bool exists = false;
	for (unsigned int i=0; i<mBodies.size(); i++)
	{
		if (mBodies[i] == body)
		{
			exists = true;
			break;
		}
	}

	if (!exists)
	{
		// Add this body..
		mBodies.push_back(body);
		// ..and add all bodies connected to it
		for (unsigned int i=0; i<body->getConnectedBodies().size(); i++)
			addAndProcessBody(body->getConnectedBodies()[i]);
	}
}

void CODEBodyIsland::move(double dx, double dy, double dz)
{
	for (unsigned int i=0; i<mBodies.size(); i++)
		mBodies[i]->move(dx, dy, dz);
}


