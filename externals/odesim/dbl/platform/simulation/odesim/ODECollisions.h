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

#ifndef ODECOLLISIONS_H_
#define ODECOLLISIONS_H_

#include <ode/ode.h>
#include <Configuration.h>
#include <SimVis.h>
#include "ODELogging.h"

// Forward declarations
class CODECollisionHandler;
class CODESim;
class CODESimAccess;

class CODEContactProps: public CODELoggable
{
	protected:
		CODECollisionHandler*	mpHandler;
		int						mMaterial1;
		int						mMaterial2;
		dSurfaceParameters		mParams;

	public:
		CODEContactProps(CODECollisionHandler* parentHandler);
		void				getParams(dSurfaceParameters *params);
		bool				isMatch(int material1, int material2);
		bool				readConfig(const CConfigSection &configSection);
};


class CODECollideInfo: public CODELoggable	// Probably not the best name in the world ...
{
	protected:
		CODECollisionHandler*	mpHandler;
		dSpaceID				mSpace1;
		dSpaceID				mSpace2;

	public:
		CODECollideInfo(CODECollisionHandler* parentHandler);

		dSpaceID			getSpace1()				{return mSpace1;}
		dSpaceID			getSpace2()				{return mSpace2;}

		bool				readConfig(const CConfigSection &configSection, CODESimAccess* simAccess);
};

// Simple, non-hierarchical collision handler.
// Add pairs of spaces to be collided using dSpaceCollide2(space1, space2).
// If both arguments are the same space, dSpaceCollide2(space, space) is equivalent to calling dSpaceCollide(space).
//
// Use addMaterialPair() to add pairs of materials to specify the type of contact joint that should be created
// when two materials collide. This function can also be called with the same material for both arguments.
class CODECollisionHandler: public CODELoggable
{
	private:
		// Static callback function
		static void		collisionCallbackFunc(void *data, dGeomID o1, dGeomID o2);

	protected:
		CODESim*						mpSim;
		dJointGroupID					mContactGroup;
		CODEContactProps				mDefaultContactProps;
		std::vector<CODEContactProps*>	mContactProps;
		std::vector<CODECollideInfo*>	mCollideInfos;
		bool							mShouldDrawContacts;
		std::vector<CSimVisObject*>		mDrawingObjects;

	protected:
		void				clearCollideInfos();
		void				clearContactProps();
		void				clearJointGroup();

	public:
		CODECollisionHandler(CODESim *parentSim);
		~CODECollisionHandler();
		// The CODECollisionHandler will be OWNER of added collideInfo and contactInfo objects!!
		void				addCollideInfo(CODECollideInfo* collideInfo);
		void				addContactProps(CODEContactProps* contactProps);

		void				calculateCollisions(CODESimAccess *simAccess);
		void				copySurfaceParameters(dSurfaceParameters* surfaceParams, CODEContactProps* contactInfo);
		CODEContactProps*	getContactProps(int material1, int material2);

		// Drawing objects that represent contacts as formed by the collision handler
		bool				shouldDrawContacts();
		void				addDrawingObject(const dContactGeom &contactGeom);
		const std::vector<CSimVisObject*>&	getDrawingObjects()	{return mDrawingObjects;}
		void				clearDrawingObjects();

		void				clearAll();

		dSpaceID			getObjectSpaceID(const std::string &objectName, CODESimAccess* simAccess);

		// Helper function, made for CODECollideInfo objects
		void				convertKDToERPCFM(double K, double D, double *ERP, double *CFM);

		// Reads configuration. Needs sim access.
		bool				readConfig(const CConfigSection &configSection, CODESimAccess* simAccess);
};


#endif /* ODECOLLISIONS_H_ */
