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

#ifndef ODEBODIES_H_
#define ODEBODIES_H_

#include <ode/ode.h>
#include <Configuration.h>
#include <vector>
#include <PlaceableObject.h>
#include <SimVis.h>
#include "ODELogging.h"

// Forward declarations
class CODEBodyAnchor;
class CODEObject;

// **************************************************************************** //
// ********************************** bodies ********************************** //
// **************************************************************************** //

class CODEBody: protected dBody, public CPlaceableObject, public CODELoggable	// Don't automatically publish all dBody methods
{
	private:
		std::vector<CODEBody*>		mConnectedBodies;	// Bodies that are connected to this body

	protected:
		// Parent object. Can be NULL, e.g., for mWorldBody in CODESim.
		CODEObject*		mpObject;
		// Center of mass & origin convention:
		// - The body has an origin/position that is not the same as the ODE origin, which must coincide with the Center of Mass (CoM)
		// - The actual ODE body will be created in the point [CMX, CMY, CMZ] relative to the ('virtual') origin
		// - The body's position, when requested, will therefore be calculated as ODE_body_position + [-CMX; -CMY; -CMZ]
		// - The drawing objects are defined relative to the origin
		std::string		mName;
		double			mMass;
		double			mIXX, mIYY, mIZZ;					// Diagonal elements of inertia matrix
		double			mIXY, mIXZ, mIYZ;					// Other elements of inertia matrix
		double			mCMX, mCMY, mCMZ;					// Center of mass
		std::vector<CODEBodyAnchor*>	mAnchors;			// Defined anchors. Can be empty. CODEBody is OWNER of the anchors and will destroy them
		void							clearAnchors();		// Clears the anchors array.

		// Drawing objects
		std::vector<CSimVisObject*>		mDrawingObjects;	// List of graphical representation objects for this body. CODEBody is OWNER
		void							clearDrawingObjects();

		void			setMass();

	public:
		CODEBody(CODEObject* pParentObject);
		virtual ~CODEBody();

		dBodyID id() const	{return dBody::id();}
		const std::string&	name() const;
		
		CODEObject*		object() { return mpObject; }

		// dx, dy and dz are displacements in the BODY'S reference frame (so taking its rotation into account)
		void			getPosition(double *pos, double dx=0, double dy=0, double dz=0)
		{
			dVector3 relPos;
			// Check if this body represents the world (ID = 0)
			if (id() != 0)
			{
				getRelPointPos(-mCMX + dx, -mCMY + dy, -mCMZ + dz, relPos);
				for (int i=0; i<3; i++)
					pos[i] = relPos[i];
			}
			else
			{
				// The world is placed at 0,0,0 AND has identity rotation matrix
				pos[0] = dx;
				pos[1] = dy;
				pos[2] = dz;
			}
		}

		void			getRotation(double *rot)
		{
			// Check if this body represents the world (ID = 0)
			if (id() != 0)
			{
				const dReal* R = dBody::getRotation();
				for (int i=0; i<12; i++)
					rot[i] = R[i];
			}
			else
			{
				// Fill the 12 elements of rot with the identity matrix
				memset(rot, 0, 12*sizeof(double));
				rot[0] = rot[5] = rot[10] = 1.0;
			}
		}

		template<class floattype>
		void			getCoMPosition(floattype *pos)	// Get position of Center of Mass
		{
			const dReal* comPos = dBody::getPosition();
			for (int i=0; i<3; i++)
				pos[i] = comPos[i];
		}

		template<class floattype>
		void			getCoMOffset(floattype *pos)	// Get position of Center of Mass
		{
			pos[0] = mCMX;
			pos[1] = mCMY;
			pos[2] = mCMZ;
		}

		// Move the body - relative displacement
		// moveIsland goes a little further and moves this object plus all connected bodies
		// dx, dy and dz are given in the GLOBAL reference frame
		void			move(double dx, double dy, double dz);
		void			moveIsland(double dx, double dy, double dz);

		// addConnectedBody() is called by joints who are actually connecting bodies
		// mConnectedBodies is NOT owner of the added bodies (of course)
		void			addConnectedBody(CODEBody* body)	{if (body->id() != 0) mConnectedBodies.push_back(body);}
		const std::vector<CODEBody*>& getConnectedBodies()	{return mConnectedBodies;}
		// disconnectBodies() does not physically disconnect them (no deletion of joints)
		// but rather clears the list of reported connected bodies.
		void			disconnectBodies()					{mConnectedBodies.clear();}

		// Define a new anchor point on the body (used for connecting bodies to each other)
		// CODEBody will become OWNER of the anchors and will destroy them
		void			addAnchor(std::string name, double x, double y, double z);
		void			addAnchor(CODEBodyAnchor* newAnchor);
		const std::vector<CODEBodyAnchor*>&		getAnchors()		{return mAnchors;}

		// Add new drawing object to this body's graphical representation
		void			addDrawingObject(CSimVisObject* newObject);
		const std::vector<CSimVisObject*>&	getDrawingObjects()	{return mDrawingObjects;}
		CSimVisObject*	resolveDrawingObject(const std::string &drawingObjectName);

		virtual void	init(dWorld& world);
		virtual void	deinit();
		virtual void    zero();
		bool			readAnchorsConfig(const CConfigSection &bodyConfigSection);			// Used for the world body, which only needs anchors and drawing objects
		bool			readDrawingObjectsConfig(const CConfigSection &bodyConfigSection);	// Used for the world body, which only needs anchors and drawing objects
		virtual bool	readConfig(const CConfigSection &configSection);
		void			setName(const std::string& name)	{mName = name;}

		const dReal *   getLinearVel() const                        {return dBody::getLinearVel();}
		const dReal *	getAngularVel() const						{return dBody::getAngularVel();}
		void			setLinearVel (const dVector3 v)				{return dBody::setLinearVel(v);}
		void			setAngularVel (dReal x, dReal y, dReal z)	{return dBody::setAngularVel(x, y, z);}

		void			setRotation(const dMatrix3 R);
		void			addForceAtRelPos(double forceX, double forceY, double forceZ, double posX, double posY, double posZ);
};

// Body anchors can be defined for CODEBody objects and can be used
// to attach two bodies to each other with a joint.
class CODEBodyAnchor: public CODELoggable
{
	protected:
		CODEBody*		mpBody;
		std::string		mName;
		double			mX, mY, mZ;	// Position of the anchor on the CODEBody, defined in the body's reference frame

	public:
		CODEBodyAnchor(CODEBody* pBody):
			mpBody(pBody)
		{
		}
		CODEBodyAnchor(CODEBody* pBody, const std::string& name, double x, double y, double z):
			mpBody(pBody),
			mName(name),
			mX(x),
			mY(y),
			mZ(z)
		{
		}

		// Get
		double				x()		{return mX;}
		double				y()		{return mY;}
		double				z()		{return mZ;}
		const std::string&	name()	const {return mName;}
		CODEBody*			body()	const {return mpBody;}
		// getPosition will retrieve the world position of the anchor point
		void				getPosition(double *pos)
		{
			mpBody->getPosition(pos, mX, mY, mZ);
		}

		// Read config
		bool				readConfig(const CConfigSection &configSection);
};

// Defines a set of interconnected bodies
class CODEBodyIsland
{
	protected:
		std::vector<CODEBody*>	mBodies;
		// Add a body to the island, including all attached bodies.
		void	addAndProcessBody(CODEBody* body);

	public:
		// Build island starting with this body
		void	buildIsland(CODEBody* body);
		void	move(double dx, double dy, double dz);
};


#endif /* ODEBODIES_H_ */
