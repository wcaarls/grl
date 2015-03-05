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

#include "SimVis.h"

void CSimVisColor::set(double r, double g, double b, double alpha)
{
	mR = (float)r;
	mG = (float)g;
	mB = (float)b;
	mAlpha = (float)alpha;
}

bool CSimVisColor::readConfig(const CConfigSection &configSection)
{
	bool configresult = true;

	if (!configSection.isNull())
	{
		configresult &= mLogAssert(configSection.get("r",  &mR));
		configresult &= mLogAssert(configSection.get("g",  &mG));
		configresult &= mLogAssert(configSection.get("b",  &mB));
		// You may leave out alpha
		configSection.get("alpha",  &mAlpha);
	}

	return configresult;
}

void CSimVisObject::getPositionF(float* pos)
{
	if (mpPlaceable != NULL)
		mpPlaceable->getPositionF(pos, mX, mY, mZ);
	else
	{
		pos[0] = (float)mX;
		pos[1] = (float)mY;
		pos[2] = (float)mZ;
	}
}

void CSimVisObject::getRotationF(float* R)
{
	if (mpPlaceable != NULL)
	{
		CODEMatrix3 rotResult;
		CODEMatrix3 rotPlaceable;
		mpPlaceable->getRotation(rotPlaceable);
		rotResult = rotPlaceable*mRotation;
		rotResult.store(R);
		//mpPlaceable->getRotationF(R);
	}
	else
	{
		//mLogDebugLn("Internal rotation matrix requested for drawobject type " << mType);
		dMatrix3 &rotation = mRotation;
        for (int i=0; i<12; i++)
			R[i] = (float)rotation[i];
	}
}

bool CSimVisObject::readConfig(const CConfigSection &configSection)
{
	bool configresult=true;

	// A name is optional and only necessary if you want to resolve the drawing object
	configSection.get("name", &mName);

	// If no color is defined, this should not turn configresult into false
	mColor.readConfig(configSection.section("color"));

	// If no position is defined, this should not turn configresult into false
	configSection.get("x",  &mX);
	configSection.get("y",  &mY);
	configSection.get("z",  &mZ);

	mRotation.readConfig(configSection);

	return configresult;
}


// Drawing objects
bool CSimVisBox::readConfig(const CConfigSection &configSection)
{
	// First, call readConfig() of the base class
	bool configresult = CSimVisObject::readConfig(configSection);

	configresult &= mLogAssert(configSection.get("dx",  &mSides[0]));
	configresult &= mLogAssert(configSection.get("dy",  &mSides[1]));
	configresult &= mLogAssert(configSection.get("dz",  &mSides[2]));

	return configresult;
}

bool CSimVisSphere::readConfig(const CConfigSection &configSection)
{
	// First, call readConfig() of the base class
	bool configresult = CSimVisObject::readConfig(configSection);

	configresult &= mLogAssert(configSection.get("radius",  &mRadius));

	return configresult;
}

bool CSimVisCylinder::readConfig(const CConfigSection &configSection)
{
	// First, call readConfig() of the base class
	bool configresult = CSimVisObject::readConfig(configSection);

	// If no position is defined, this should not turn configresult into false
	configresult &= mLogAssert(configSection.get("length",  &mLength));
	configresult &= mLogAssert(configSection.get("radius",  &mRadius));

	return configresult;
}

// ******************************** vertex strip ******************************** //
void CSimVisStrip::allocateVertices(int numVertices)
{
	if (mData != NULL)
		delete[] mData;
	mNumVertices = numVertices;
	mData = new float[numVertices*3];
}

void CSimVisStrip::setVertex(int index, float *v)
{
	memcpy(mData + index*3, v, 3*sizeof(float));
	//mLogInfoLn("Added vertex: X:" << v[0] << "\tY:" << v[1] << "\tZ:" << v[2]);
}

// ***************************** //
// Global class factory function //
// ***************************** //

CSimVisObject* gCreateSimVisObject(const std::string& objectTypeStr, CPlaceableObject* parentPlaceableObject)
{
	CSimVisObject* newObject = NULL;

	if (objectTypeStr == "box")
		newObject = new CSimVisBox(parentPlaceableObject);
	else if (objectTypeStr == "sphere")
		newObject = new CSimVisSphere(parentPlaceableObject);
	else if (objectTypeStr == "cylinder")
		newObject = new CSimVisCylinder(parentPlaceableObject);
	else if (objectTypeStr == "capsule")
		newObject = new CSimVisCapsule(parentPlaceableObject);
	else if (objectTypeStr == "cone")
		newObject = new CSimVisCone(parentPlaceableObject);
	else if (objectTypeStr == "trianglestrip")
		newObject = new CSimVisStrip(parentPlaceableObject);

	return newObject;
}

