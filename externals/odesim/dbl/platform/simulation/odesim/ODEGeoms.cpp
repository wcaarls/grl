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

#include "ODEGeoms.h"
#include "ODEObjects.h"

CODEGeom::CODEGeom():
	mID(0)
{
}

CODEGeom::~CODEGeom()
{
	clearReportedContacts();

	if (mID)
		dGeomDestroy(mID);
}

bool CODEGeom::init(dSpaceID space)
{
	create(space);
	dGeomSetData(mID, this);

	if (mID != 0)
		mLogDebugLn("Created " << getClassName() << "-geom \"" << mName << "\"!");
	else
		mLogErrorLn("Trying to initialize geom that failed to be created! Check your configuration.");

	return mID != 0;
}

std::string CODEGeom::getClassName() const
{
	switch(getClass())
	{
		case  dSphereClass:
			return "sphere";
		case  dBoxClass:
			return "box";
		case  dCapsuleClass:
			return "capsule";
		case  dCylinderClass:
			return "cylinder";
		case  dPlaneClass:
			return "plane";
		case  dRayClass:
			return "ray";
		case  dConvexClass:
			return "convex";
		case  dGeomTransformClass:
			return "geomtransform";
		case  dTriMeshClass:
			return "trimesh";
		case  dHeightfieldClass:
			return "heightfield";

		case  dSimpleSpaceClass:
			return "simplespace";
		case  dHashSpaceClass:
			return "hashspace";
		case  dSweepAndPruneSpaceClass:
			return "sweepandprunespace";
		case  dQuadTreeSpaceClass:
			return "quadtreespace";
		default:
			return "unknown";
	}
}

bool CODEGeom::readConfig(const CConfigSection &configSection)
{
	bool configresult = true;

	// Read name
	configresult &= mLogAssert(configSection.get("name", &mName, "nameless-geom"));

	// Read material
	configresult &= mLogAssert(configSection.get("material", &mMaterial));

	return configresult;
}

void CODEGeom::reportContact(CODEGeom *collidedObject, const dVector3 &pos)
{
	mContacts.push_back(new CODEGeomContact(collidedObject, pos));
}

void CODEGeom::clearReportedContacts()
{
	for (unsigned int iContact=0; iContact<mContacts.size(); iContact++)
		delete mContacts[iContact];

	mContacts.clear();
}

CODEPlaceableGeom::CODEPlaceableGeom(CODEObject *parentObject):
	CODEGeom(parentObject),
	mpBody(NULL),
	mpGeomTransform(NULL),
	mX(0), mY(0), mZ(0)
{
	dRSetIdentity(mRotation);
}

CODEPlaceableGeom::~CODEPlaceableGeom()
{
	// Cleanup geomtransform
	if (mpGeomTransform != NULL)
	{
		delete mpGeomTransform;
		mpGeomTransform = NULL;
	}
	// Cleanup drawing objects
	clearDrawingObjects();
}


void CODEPlaceableGeom::setPosition(dReal x, dReal y, dReal z)
{
	dGeomSetPosition(mID, x, y, z);
	if (mpObject != NULL)
		mpObject->invalidateCollisions();
}

void CODEPlaceableGeom::setRotation(const dMatrix3 R)
{
	dGeomSetRotation(mID, R);
	if (mpObject != NULL)
		mpObject->invalidateCollisions();
}

void CODEPlaceableGeom::setQuaternion(const dQuaternion quat)
{
	dGeomSetQuaternion(mID, quat);
	if (mpObject != NULL)
		mpObject->invalidateCollisions();
}

void CODEPlaceableGeom::getPosition(double *pos, double dx, double dy, double dz)
{
	if (mpGeomTransform != NULL)
	{
		mpGeomTransform->getPositionEncapsulated(pos, dx, dy, dz);
		//dbgprintf("[DEBUG] Position of placeable geom calculated as getPositionEncapsulated.\n");
	}
	else
	{
		const dReal *dPos	= dGeomGetPosition(mID);
		const dReal *dR		= dGeomGetRotation(mID);
		// Get the transformed displacement of dx, dy and dz in the geom's reference frame
		dVector3 dPosDisp = {dx, dy, dz, 1.0};
		dVector3 dRelPos;
		dMULTIPLY0_331(dRelPos, dR, dPosDisp);
		pos[0] = dRelPos[0] + dPos[0];
		pos[1] = dRelPos[1] + dPos[1];
		pos[2] = dRelPos[2] + dPos[2];
	}
}

void CODEPlaceableGeom::getRotation(double *rot)
{
	if (mpGeomTransform != NULL)
		mpGeomTransform->getRotationEncapsulated(rot);
	else
	{
		const dReal *dR = dGeomGetRotation(mID);
		for (int i=0; i<12; i++)
			rot[i] = dR[i];
	}
}

void CODEPlaceableGeom::clearDrawingObjects()
{
	for (unsigned int i=0; i<mDrawingObjects.size(); i++)
		delete mDrawingObjects[i];

	mDrawingObjects.clear();
}

void CODEPlaceableGeom::addDrawingObject(CSimVisObject* newObject)
{
	mDrawingObjects.push_back(newObject);
}

bool CODEPlaceableGeom::readConfig(const CConfigSection &configSection)
{
	bool configresult = true;

	configresult &= CODEGeom::readConfig(configSection);

	// Read body. If a bodyname is defined and resolved, ALWAYS create a geomtransform object.
	std::string bodyName;
	bool bodyfound = configSection.get("bodyname", &bodyName);
	if (bodyfound)
	{
		mpBody = mpObject->resolveBody(bodyName);
		if (mpBody == NULL)
		{
			mLogErrorLn("Could not resolve body \"" << bodyName << "\" for placeable geom in object \"" << mpObject->name() << "\"!");
			configresult = false;
		}
		else
		{
			mLogDebugLn("A placeable geom will be attached to body \"" << bodyName << "\"; now adding geomtransform.");

			if (mpGeomTransform != NULL)
				delete mpGeomTransform;
			mpGeomTransform = new CODEGeomTransform(mpObject);
		}
	}

	// Read position
	configresult &= mLogAssert(configSection.get("x", &mX));
	configresult &= mLogAssert(configSection.get("y", &mY));
	configresult &= mLogAssert(configSection.get("z", &mZ));

	// Read orientation
	mRotation.readConfig(configSection);

	// Note: don't read drawing objects. Geoms should be drawn as they are; a drawingobject is added in a geom's create() function.

	return configresult;
}

bool CODEPlaceableGeom::init(dSpaceID space)
{
	bool result = true;
	if (mpGeomTransform != NULL)
	{
		if (!CODEGeom::init(NULL))	// put the geomtransform in the space, NOT the object itself
		{
			result = false;
		}
		else
		if (mpGeomTransform->init(space))
		{
			mpGeomTransform->setGeom(this);
			result = true;
		}
		else
			result = false;
	}
	else
	{
		result = CODEGeom::init(space);
	}

	if (result)
	{
		// If there's a body, this means there's a geomtransform.
		// In that case, attach the geomtransform to the body.
		if (mpBody)
		{
			mpGeomTransform->setBody(mpBody->id());
			double comPos[3];
			mpBody->getCoMOffset(comPos);
			dGeomSetPosition(id(), mX -comPos[0], mY -comPos[1], mZ -comPos[2]);
			dGeomSetRotation(id(), mRotation);
		}
		else
		{
			// If there's no body, just set position and rotation of the geom itself
			dGeomSetPosition(id(), mX, mY, mZ);
			dGeomSetRotation(id(), mRotation);
		}
	}

	return result;
}

CODEGeomTransform::CODEGeomTransform(CODEObject *parentObject):
	CODEPlaceableGeom(parentObject)
{
	mName = "transform";
}

void CODEGeomTransform::create(dSpaceID space)
{
	mID = dCreateGeomTransform(space);
	dGeomTransformSetInfo(mID, 0);	// Set information mode to '0'.
}

void CODEGeomTransform::setGeom(CODEPlaceableGeom* geom)
{
	dGeomTransformSetGeom(mID, geom->id());
}

dGeomID CODEGeomTransform::getGeomID()
{
	return dGeomTransformGetGeom(mID);
}

CODEGeom* CODEGeomTransform::getGeom()
{
	return (CODEGeom*)dGeomGetData(getGeomID());
}

void CODEGeomTransform::getPosition(double *pos, double dx, double dy, double dz)
{
	const dReal *dPos	= dGeomGetPosition(mID);
	// TODO: make displacement support for dx, dy, dz (see getPositionEncapsulated for an example). Is this function ever used anyway??
	pos[0] = dPos[0];
	pos[1] = dPos[1];
	pos[2] = dPos[2];
}

void CODEGeomTransform::getRotation(double *rot)
{
	const dReal *dR = dGeomGetRotation(mID);
	for (int i=0; i<12; i++)
		rot[i] = dR[i];
}

void CODEGeomTransform::getPositionEncapsulated(double *pos, double dx, double dy, double dz)
{
	const dReal *dPos		= dGeomGetPosition(mID);
	const dReal *dR			= dGeomGetRotation(mID);
	// Get encapsulated geometry gEnc
	dVector3 dPosEnc;
	memcpy(dPosEnc, dGeomGetPosition(getGeomID()), sizeof(dPosEnc));	// Position of encapsulated geom in the original geom's reference frame
	dPosEnc[0] += dx;
	dPosEnc[1] += dy;
	dPosEnc[2] += dz;
	dVector3 dRelPos;
	dMULTIPLY0_331(dRelPos, dR, dPosEnc);
	pos[0] = dRelPos[0] + dPos[0];
	pos[1] = dRelPos[1] + dPos[1];
	pos[2] = dRelPos[2] + dPos[2];
}

void CODEGeomTransform::getRotationEncapsulated(double *rot)
{
	const dReal *dR		= dGeomGetRotation(mID);
	const dReal *dREnc	= dGeomGetRotation(getGeomID());	// Rotation of encapsulated geom
	dMatrix3 dRelR;
	dMULTIPLY0_333 (dRelR, dR, dREnc);
	for (int i=0; i<12; i++)
		rot[i] = dRelR[i];
}

CODESphereGeom::CODESphereGeom(CODEObject *parentObject):
	CODEPlaceableGeom(parentObject),
	mRadius(1.0)
{
}

void CODESphereGeom::create(dSpaceID space)
{
	mID = dCreateSphere(space, mRadius);
	// Also create drawingobject
	CSimVisSphere *drawingSphere = new CSimVisSphere(this);
	drawingSphere->mRadius = (float)mRadius;
	drawingSphere->mColor.mAlpha = 0.5;
	mDrawingObjects.push_back(drawingSphere);
}

bool CODESphereGeom::readConfig(const CConfigSection &configSection)
{
	bool configresult = true;

	configresult &= CODEPlaceableGeom::readConfig(configSection);

	// Read parameters
	configresult &= mLogAssert(configSection.get("radius", &mRadius));
	//dbgprintf("[DEBUG] CODESphereGeom radius is now %.3f\n", mRadius);

	return configresult;
}

CODEBoxGeom::CODEBoxGeom(CODEObject *parentObject):
	CODEPlaceableGeom(parentObject),
	mDx(1.0), mDy(1.0), mDz(1.0)
{
}

void CODEBoxGeom::create(dSpaceID space)
{
	mID = dCreateBox(space, mDx, mDy, mDz);
	// Also create drawingobject
	CSimVisBox *drawingBox = new CSimVisBox(this);
	drawingBox->mSides[0] = (float)mDx;
	drawingBox->mSides[1] = (float)mDy;
	drawingBox->mSides[2] = (float)mDz;
	drawingBox->mColor.mAlpha = 0.5;
	mDrawingObjects.push_back(drawingBox);
}

bool CODEBoxGeom::readConfig(const CConfigSection &configSection)
{
	bool configresult = true;

	configresult &= CODEPlaceableGeom::readConfig(configSection);

	// Read parameters
	configresult &= mLogAssert(configSection.get("dx", &mDx));
	configresult &= mLogAssert(configSection.get("dy", &mDy));
	configresult &= mLogAssert(configSection.get("dz", &mDz));

	return configresult;
}

CODECylinderGeom::CODECylinderGeom(CODEObject *parentObject):
	CODEPlaceableGeom(parentObject),
	mRadius(1.0), mLength(1.0)
{

}

void CODECylinderGeom::create(dSpaceID space)
{
	mID = dCreateCylinder(space, mRadius, mLength);
	// Also create drawingobject
	CSimVisCylinder *drawingCylinder = new CSimVisCylinder(this);
	drawingCylinder->mRadius = (float)mRadius;
	drawingCylinder->mLength = (float)mLength;
	drawingCylinder->mColor.mAlpha = 0.5;
	mDrawingObjects.push_back(drawingCylinder);
}

bool CODECylinderGeom::readConfig(const CConfigSection &configSection)
{
	bool configresult = true;

	configresult &= CODEPlaceableGeom::readConfig(configSection);

	// Read parameters
	configresult &= mLogAssert(configSection.get("radius", &mRadius));
	configresult &= mLogAssert(configSection.get("length", &mLength));

	return configresult;
}

CODECapsuleGeom::CODECapsuleGeom(CODEObject *parentObject):
	CODECylinderGeom(parentObject)
{

}

void CODECapsuleGeom::create(dSpaceID space)
{
	mID = dCreateCapsule(space, mRadius, mLength);
	// Also create drawingobject
	CSimVisCapsule *drawingCapsule = new CSimVisCapsule(this);
	drawingCapsule->mRadius = (float)mRadius;
	drawingCapsule->mLength = (float)mLength;
	drawingCapsule->mColor.mAlpha = 0.5;
	mDrawingObjects.push_back(drawingCapsule);
}

CODEPlaneGeom::CODEPlaneGeom(CODEObject *parentObject):
	CODEGeom(parentObject),
	mA(0), mB(0), mC(1.0), mD(0)
{
}

void CODEPlaneGeom::create(dSpaceID space)
{
	mID = dCreatePlane(space, mA, mB, mC, mD);
}

bool CODEPlaneGeom::readConfig(const CConfigSection &configSection)
{
	bool configresult = true;

	configresult &= CODEGeom::readConfig(configSection);

	// Read parameters
	configresult &= mLogAssert(configSection.get("A", &mA));
	configresult &= mLogAssert(configSection.get("B", &mB));
	configresult &= mLogAssert(configSection.get("C", &mC));
	configresult &= mLogAssert(configSection.get("D", &mD));

	return configresult;
}

// ***************** heightfield ******************* //
CODEHeightfield::CODEHeightfield(CODEObject *parentObject):
	CODEPlaceableGeom(parentObject),
	mData(NULL),
	mHeightData(NULL),
	mNumX(0),
	mNumZ(0),
	mDx(0),
	mDz(0),
	mWrap(false)
{
	mData = dGeomHeightfieldDataCreate();
}

CODEHeightfield::~CODEHeightfield()
{
	dGeomHeightfieldDataDestroy(mData);
	if (mHeightData != NULL)
		delete[] mHeightData;
}

bool CODEHeightfield::readConfig(const CConfigSection &configSection)
{
	std::string heightFuncStr;

	bool configresult = true;
	configresult &= CODEPlaceableGeom::readConfig(configSection);

	configresult &= mLogAssert(configSection.get("dx", &mDx));
	configresult &= mLogAssert(configSection.get("dz", &mDz));
	configresult &= mLogAssert(configSection.get("numx", &mNumX));
	configresult &= mLogAssert(configSection.get("numz", &mNumZ));
	configSection.get("wrap", &mWrap);

	configSection.get("heightfunc", &heightFuncStr);

	//create height data memory
	if (mHeightData != NULL)
		delete[] mHeightData;
	mHeightData = new double[mNumX*mNumZ];

	// If a height function is given, generate height data
	if (!heightFuncStr.empty())
	{
		// The covered area is [-mDx/2, mDx/2] x [-mDz/2, mDz/2].
		mu::Parser parser;
		parser.SetExpr(heightFuncStr);
		double x=0, z=0;
		try
		{
			double xstep = mDx/(mNumX-1);
			double zstep = mDz/(mNumZ-1);
			for (int iX=0; iX<mNumX; iX++)
			{
				x = iX*xstep - mDx/2;
				parser.DefineConst("x", x);
				for (int iZ=0; iZ<mNumZ; iZ++)
				{
					z = iZ*zstep - mDz/2;
					parser.DefineConst("z", z);
					mHeightData[iX + iZ*mNumX] = parser.Eval();
				}
			}
		}
		catch (mu::Parser::exception_type &e)
		{
			mLogErrorLn("Unable to resolve initialization expression \"" << heightFuncStr << "\"(x=" << x << ", z=" << z << "): " << e.GetMsg());
			configresult =  false;
		}
	}
	else
	{
		// Read height data per point
		CConfigSection dataNode = configSection.section("heightdata");
		if (dataNode.isNull())
		{
			mLogErrorLn("Unable to generate or retrieve height data for heightfield geom!");
			configresult =  false;
		}
		else
		{
			int iVertex=0;
			for (CConfigProperty h = dataNode.firstProperty(); !h.isNull(); h = h.nextProperty())
			{
				// Only process properties named "h" and "hrow"
				if (h.name() == "h")
				{
					if (iVertex < mNumX*mNumZ)
						mHeightData[iVertex++] = h.toFloat();
					else
						iVertex++;	// Count how many superfluous data points were provided
				}
				else if (h.name() == "hrow")
				{
					// Process data array
					CConfigPropertyArray row;
					h.toArray(&row);
					for (unsigned int iH=0; iH<row.size(); iH++)
					{
						if (iVertex < mNumX*mNumZ)
							mHeightData[iVertex++] = row[iH].toFloat();
						else
							iVertex++;	// Count how many superfluous data points were provided
					}
				}
			}
			if (iVertex != mNumX*mNumZ)
			{
				mLogErrorLn("Found " << iVertex << " datapoints in \"heightData\" node, but expected " << mNumX*mNumZ);
				configresult = false;
			}
		}
	}
	// Delete heightdata if not read successfully
	if (!configresult)
	{
		// Fail
		delete[] mHeightData;
		mHeightData = NULL;
	}
	else
	{
		// Success!
		dGeomHeightfieldDataBuildDouble(mData, mHeightData, 0, mDx, mDz, mNumX, mNumZ, 1.0, 0.0, 0.0, mWrap);
	}
	return configresult;
}

void CODEHeightfield::create(dSpaceID space)
{
	mID = dCreateHeightfield(space, mData, 1);	// The last parameter nonzero means 'placeable'
	// Also create drawingobject if heightData was successfully read
	if (mHeightData != NULL)
	{
		CSimVisStrip *drawingStrip = new CSimVisStrip(this);
		drawingStrip->mColor.mAlpha = 0.8;
		// Add vertex data. The covered area is [-mDx/2, mDx/2] x [-mDz/2, mDz/2].
		const int numVertices = 2*mNumX*(mNumZ-1) + (mNumZ-2)*2 + 1;
		drawingStrip->allocateVertices(numVertices);
		int iV=0;
		float v[3] = {0, 0, 0};
		double xstep = mDx/(mNumX-1);
		double zstep = mDz/(mNumZ-1);
		for (int iZ=0; iZ<mNumZ-1; iZ++)
		{
			for (int iX=0; iX<mNumX; iX++)
			{
				v[0] = iX*xstep - mDx/2;
				v[1] = mHeightData[iX + iZ*mNumX];
				v[2] = iZ*zstep - mDz/2;
				drawingStrip->setVertex(iV++, v);
				v[1] = mHeightData[iX + (iZ+1)*mNumX];
				v[2] = (iZ+1)*zstep - mDz/2;
				drawingStrip->setVertex(iV++, v);
			}
			// Stitch if necessary
			// Add last point again - we do this always (also at the very end). Otherwise the fore-last triangle has an inverted normal (weird bug).
			drawingStrip->setVertex(iV++, v);
			// Add first point of next substrip - only when there's another substrip coming
			if (iZ < mNumZ-2)
			{
				v[0] = 0 - mDx/2;
				v[1] = mHeightData[ 0 + (iZ+1)*mNumX];
				//v[2] = (iZ+1)*zstep - mDz/2;
				drawingStrip->setVertex(iV++, v);
			}
		}
		mLogAssert(iV++ == numVertices);
		// Push drawing object onto the vector
		mDrawingObjects.push_back(drawingStrip);
	}
}

// ***************************** //
// Global class factory function //
// ***************************** //

CODEGeom* gODECreateGeom(const std::string& geomTypeStr, CODEObject* parentObject)
{
	CODEGeom* newGeom = NULL;

	if (geomTypeStr == "sphere")
		newGeom = new CODESphereGeom(parentObject);
	else if (geomTypeStr == "box")
		newGeom = new CODEBoxGeom(parentObject);
	else if (geomTypeStr == "cylinder")
		newGeom = new CODECylinderGeom(parentObject);
	else if (geomTypeStr == "capsule")
		newGeom = new CODECapsuleGeom(parentObject);
	else if (geomTypeStr == "plane")
		newGeom = new CODEPlaneGeom(parentObject);
	else if (geomTypeStr == "heightfield")
		newGeom = new CODEHeightfield(parentObject);
	else
		logErrorLn(CLog2("ode"), "Unknown geom type " << geomTypeStr << "!");

	return newGeom;
}

