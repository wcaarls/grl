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

#ifndef SIMVIS_H_
#define SIMVIS_H_

#include <Configuration.h>
#include <Log2.h>
#include <GenericSim.h>
#include <PlaceableObject.h>
#include <ODEMatrix.h>	// TODO: remove dependency on ODE by replacing CODEMatrix3!!!

class CSimVisLoggable
{
	protected:
		CLog2	mLog;
	public:
		CSimVisLoggable():	mLog("simvis")	{}
};

enum ESimVisObjectType
{
	dtBox,
	dtSphere,
	dtCylinder,
	dtCapsule,
	dtCone,
	dtTriangleStrip
};

class CSimVisColor: public CSimVisLoggable
{
	public:
		float	mR, mG, mB, mAlpha;

		CSimVisColor():
			mR(0.6f), mG(0.1f), mB(0.1f), mAlpha(1.0f)
			{}

		void	set(double r, double g, double b, double alpha=1.0);
		bool	readConfig(const CConfigSection &configSection);
};

// Object that can be drawn by the CSimVisWidget
class CSimVisObject: public CSimVisLoggable
{
	protected:
		std::string				mName;
		CPlaceableObject		*mpPlaceable;	// parent placeable object
		ESimVisObjectType		mType;

	public:
		double					mX, mY, mZ;	// Position within the placeable object's reference frame
		CODEMatrix3				mRotation;	// Rotation matrix
		// color
		CSimVisColor		mColor;

		CSimVisObject(CPlaceableObject* parentPlaceableObject, ESimVisObjectType type):
			mpPlaceable(parentPlaceableObject),
			mType(type), mX(0), mY(0), mZ(0)
		{
			dRSetIdentity(mRotation);
		}
		virtual ~CSimVisObject()	{}

		const std::string&		name() const				{return mName;}

		// Override these to place the object in its desired position
		// dx, dy and dz are displacements in the CPlaceableObject's reference frame!
		virtual void			getPositionF(float* pos);
		virtual void			getRotationF(float* R);

		// Override these to pass type-specific information to the drawing widget
		virtual float			getParam1()					{return 0;}
		virtual float			getParam2()					{return 0;}
		virtual float			getParam3()					{return 0;}
		//virtual float	getParam4()	{return 0;}	// Not used right now

		// Override these functions for vertex-type drawing objects
		virtual float*			getVertexData()				{return NULL;}
		virtual int				getNumVertices()			{return 0;}

		ESimVisObjectType		getType()					{return mType;}

		virtual bool			readConfig(const CConfigSection &configSection);

};

typedef std::vector<CSimVisObject*> CSimVisObjectPtrArray;

class CSimVisBox: public CSimVisObject
{
	public:
		float	mSides[3];
		CSimVisBox(CPlaceableObject* parentPlaceableObject):
			CSimVisObject(parentPlaceableObject, dtBox)
		{
		}
		float	getParam1()	{return mSides[0];}
		float	getParam2()	{return mSides[1];}
		float	getParam3()	{return mSides[2];}

		bool	readConfig(const CConfigSection &configSection);
};

class CSimVisSphere: public CSimVisObject
{
	public:
		float	mRadius;
		CSimVisSphere(CPlaceableObject* parentPlaceableObject):
			CSimVisObject(parentPlaceableObject, dtSphere),
			mRadius(1.0)
		{
		}
		float	getParam1()	{return mRadius;}

		bool	readConfig(const CConfigSection &configSection);
};

class CSimVisCylinder: public CSimVisObject
{
	public:
		float	mLength;
		float	mRadius;
		CSimVisCylinder(CPlaceableObject* parentPlaceableObject):
			CSimVisObject(parentPlaceableObject, dtCylinder),
			mLength(1.0),
			mRadius(1.0)
		{
		}
		float	getParam1()	{return mLength;}
		float	getParam2()	{return mRadius;}

		bool	readConfig(const CConfigSection &configSection);
};

class CSimVisCapsule: public CSimVisCylinder
{
	public:
		CSimVisCapsule(CPlaceableObject* parentPlaceableObject):
			CSimVisCylinder(parentPlaceableObject)
		{
			mType = dtCapsule;
		}
};

class CSimVisCone: public CSimVisCylinder
{
	public:
		CSimVisCone(CPlaceableObject* parentPlaceableObject):
			CSimVisCylinder(parentPlaceableObject)
		{
			mType = dtCone;
		}
};

// Draw a triangle strip from a list of vertices
class CSimVisStrip: public CSimVisObject
{
	protected:
		int		mNumVertices;
		float	*mData;	// Vertex data of the form [x0 y0 z0 x1 y1 z1 ...]
	public:
		CSimVisStrip(CPlaceableObject* parentPlaceableObject):
			CSimVisObject(parentPlaceableObject, dtTriangleStrip),
			mNumVertices(0),
			mData(NULL)
		{
		}
		~CSimVisStrip()
		{
			if (mData != NULL)
				delete[] mData;
		}

		void			allocateVertices(int numVertices);
		void			setVertex(int index, float *v);

		virtual float*	getVertexData()			{return mData;}
		virtual int		getNumVertices()		{return mNumVertices;}
};

// Global class factory function
CSimVisObject* gCreateSimVisObject(const std::string& objectTypeStr, CPlaceableObject* parentPlaceableObject);



// Base class for visualizable simulator
class CVisualSim: public CGenericSim
{
	public:
		// TODO: perhaps move this function to simaccess? But then we need to acquire a derived sim access object, which is more tedious to implement.
		virtual void	getSimVisObjects(CSimVisObjectPtrArray* objects)=0;
};

/*
class CVisualSimAccess: public CGenericSimAccess
{
	public:
		CVisualSimAccess():
			CGenericSimAccess()		{}
		CVisualSimAccess(CVisualSim* sim):
			CGenericSimAccess(sim)	{}

};
*/
#endif /* SIMVIS_H_ */
