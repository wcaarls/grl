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

#ifndef ODEGEOMS_H_
#define ODEGEOMS_H_

#include <ode/ode.h>
#include <Configuration.h>
#include <SimVis.h>
#include "ODEMatrix.h"
#include "ODEBodies.h"
#include "ODELogging.h"
//#include <vector>


// Forward declaration
class CODEObject;
class CODEGeom;

#define ODE_GEOM_DEFAULT_MATERIAL	-1

class CODEGeomContact
{
	protected:
		CODEGeom*	mpGeom;
		double		mX, mY, mZ;

	public:
		CODEGeomContact(CODEGeom* collidedGeom, const dVector3 &pos):
			mpGeom(collidedGeom)
		{
			mX = pos[0];
			mY = pos[1];
			mZ = pos[2];
		}

		double		getX()		{return mX;}
		double		getY()		{return mY;}
		double		getZ()		{return mZ;}
		CODEGeom*	getGeom()	{return mpGeom;}
};

// **************************************************************************** //
// ******************************** geometries ******************************** //
// **************************************************************************** //

// I don't like the class structure in odecpp_collision.h for geoms, so I made my own.
// For example, create() is not a virtual function in dGeom so it cannot be called
// from a base class pointer dGeom*.
//
class CODEGeom: public CODELoggable
{
	private:
		// intentionally undefined, don't use these
		CODEGeom (CODEGeom &);
		void operator= (CODEGeom &);

	protected:
		dGeomID mID;
		std::string		mName;
		CODEObject*		mpObject;	// Parent object. Used to resolve body anchors
		CODEGeom();		// Don't use CODEGeom directly; derive a new class
		virtual void	create(dSpaceID space)=0;	// Abstract; you must implement it
		int				mMaterial;	// Material identifier used when creating contact joints between two materials
		std::vector<CODEGeomContact*>	mContacts;	// Current contact points, as reported by a collision handler

	public:
		CODEGeom(CODEObject *parentObject):
			mID(0),
			mName("nameless-geom"),
			mpObject(parentObject),
			mMaterial(ODE_GEOM_DEFAULT_MATERIAL)
		{
		}
		virtual			~CODEGeom();

		const std::string&	name() const							{return mName;}
		virtual bool		init(dSpaceID space);
		virtual void		deinit()								{}
		virtual bool		readConfig(const CConfigSection &configSection);
		virtual dBodyID		getBody() const							{return NULL;}	// a non-placeable geom such as CODEGeom should return NULL, which refers to the static environment body.
		dGeomID				id() const								{return mID;}
		operator			dGeomID() const							{return mID;}

		void				destroy()								{if (mID) dGeomDestroy(mID); mID = 0;}

		int					getClass() const						{return dGeomGetClass(mID);}
		std::string			getClassName() const;
		virtual bool		isPlaceable() const						{return false;}
		virtual int			getMaterial() const						{return mMaterial;}

		void				setData(void *data)						{dGeomSetData(mID, data);}
		void*				getData() const							{return dGeomGetData(mID);}

		void				reportContact(CODEGeom *collidedObject, const dVector3 &pos);
		void				clearReportedContacts();
		const std::vector<CODEGeomContact*>&	getContacts()	{return mContacts;}
};

// Forward declaration
class CODEGeomTransform;

class CODEPlaceableGeom: public CODEGeom, public CPlaceableObject
{
	protected:
		CODEBody*			mpBody;				// If a body is found in the config, a geomtransform is ALWAYS created. "if body then geomtransform"
		CODEGeomTransform*	mpGeomTransform;
		double				mX, mY, mZ;			// Position of the geom. If attached to a body, this is relative to the bodies 'virtual' origin, NOT to the body's center of mass.
		CODEMatrix3			mRotation;			// Rotation of the geom. If attached to a body, this is relative to the bodies 'virtual' origin.

		// Drawing objects
		std::vector<CSimVisObject*>		mDrawingObjects;	// List of graphical representation objects for this body. CODEBody is OWNER
		void							clearDrawingObjects();

	public:
		CODEPlaceableGeom(CODEObject *parentObject);
		~CODEPlaceableGeom();

		virtual bool	init(dSpaceID space);
		virtual bool	isPlaceable() const						{return true;}

		void			setBody(dBodyID b)						{dGeomSetBody(mID, b);}
		dBodyID			getBody() const							{if(mpBody != NULL) return mpBody->id(); else return NULL;}	// This always returns the body of this geom, even if it is encapsulated by a geomtransform!

		void			setPosition(dReal x, dReal y, dReal z);
		void			getPosition(double *pos, double dx=0, double dy=0, double dz=0);
		//const dReal*	getPosition() const						{return dGeomGetPosition(mID);}

		void			setRotation(const dMatrix3 R);
		void			getRotation(double *rot);
		//const dReal*	getRotation() const						{return dGeomGetRotation(mID);}

		void			setQuaternion(const dQuaternion quat);
		void			getQuaternion (dQuaternion quat) const	{dGeomGetQuaternion(mID, quat);}

		virtual bool	readConfig(const CConfigSection &configSection);

		// Add new drawing object to this body's graphical representation
		void			addDrawingObject(CSimVisObject* newObject);
		const std::vector<CSimVisObject*>&	getDrawingObjects()	{return mDrawingObjects;}
};

class CODEGeomTransform: public CODEPlaceableGeom
{
	protected:
		void			create(dSpaceID space);

	public:
		CODEGeomTransform(CODEObject *parentObject);
		void			setGeom(CODEPlaceableGeom* geom);
		dGeomID			getGeomID();
		CODEGeom*		getGeom();

		void			getPosition(double *pos, double dx=0, double dy=0, double dz=0);
		void			getRotation(double *rot);

		// Handy functions that calculate the final pos and R of the object that is encapsulated by this geomtransform
		void			getPositionEncapsulated(double *pos, double dx=0, double dy=0, double dz=0);
		void			getRotationEncapsulated(double *rot);

};

class CODESphereGeom: public CODEPlaceableGeom
{
	protected:
		double			mRadius;
		void			create(dSpaceID space);

	public:
		CODESphereGeom(CODEObject *parentObject);
		bool			readConfig(const CConfigSection &configSection);

};

class CODEBoxGeom: public CODEPlaceableGeom
{
	protected:
		double			mDx, mDy, mDz;
		void			create(dSpaceID space);

	public:
		CODEBoxGeom(CODEObject *parentObject);
		bool			readConfig(const CConfigSection &configSection);

};

class CODECylinderGeom: public CODEPlaceableGeom
{
	protected:
		double			mRadius;
		double			mLength;
		void			create(dSpaceID space);

	public:
		CODECylinderGeom(CODEObject *parentObject);
		bool			readConfig(const CConfigSection &configSection);

};
class CODECapsuleGeom: public CODECylinderGeom
{
	protected:
		void			create(dSpaceID space);

	public:
		CODECapsuleGeom(CODEObject *parentObject);

};

// A plane is a non-placeable geom
class CODEPlaneGeom: public CODEGeom
{
	protected:
		double			mA, mB, mC, mD;
		void			create(dSpaceID space);

	public:
		CODEPlaneGeom(CODEObject *parentObject);
		bool			readConfig(const CConfigSection &configSection);

};

class CODEHeightfield: public CODEPlaceableGeom
{
	protected:
		dHeightfieldDataID	mData;
		double				*mHeightData;
		int					mNumX;
		int					mNumZ;
		double				mDx;
		double				mDz;
		bool				mWrap;
		void				create(dSpaceID space);

	public:
		CODEHeightfield(CODEObject *parentObject);
		~CODEHeightfield();
		bool				readConfig(const CConfigSection &configSection);
};

// Global class factory function //
CODEGeom* gODECreateGeom(const std::string& geomTypeStr, CODEObject* parentObject);

#endif /* ODEGEOMS_H_ */
