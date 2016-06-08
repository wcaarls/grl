/*
 *    Classes to support ODE objects
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

#ifndef __ODEOBJECTS_H_INCLUDED
#define __ODEOBJECTS_H_INCLUDED

#include <ode/ode.h>
#include <Configuration.h>
#include <vector>

#include <grl/utils.h>

#include "ODEBodies.h"
#include "ODEJoints.h"
#include "ODEGeoms.h"
#include "ODELogging.h"

// Forward declaration of CODEObject
class CODEObject;

class CODEInitializationExpression : public CODELoggable
{
  private:
    std::string mExpression;
    static mu::Parser mParser;
    static grl::Rand mRand;

  public:
    CODEInitializationExpression(const std::string &expression="0") : mExpression(expression) { }

    void setExpression(const std::string &expression)
    {
      mExpression = expression;
    }

    static void randomInit(long int seed)
    {
      if (seed > 0)
        mRand.init(seed);
    }

    double evaluate()
    {
      mParser.SetExpr(mExpression);
      mParser.DefineConst("x", mRand.get());

      try
      {
        double x = mParser.Eval();
        //mLogDebugLn("Initialization expression " << mExpression << " evaluated to " << x);
        return x;
      }
      catch (mu::Parser::exception_type &e)
      {
        mLogErrorLn("Unable to resolve initialization expression \"" << mExpression << "\": " << e.GetMsg());
        return 0;
      }
    }
};

class CODEObjectFixedPoint: public CODELoggable
{
	protected:
		std::string		mBodyName;
		double mWorldX, mWorldY, mWorldZ;
		double mBodyX, mBodyY, mBodyZ;

		CODEInitializationExpression mWorldXExpr, mWorldYExpr, mWorldZExpr;

	public:
		CODEObjectFixedPoint();

		const std::string&	getBodyName()	{return mBodyName;}
		double				getWorldX()   {return mWorldX;}
		double				getWorldY()   {return mWorldY;}
		double				getWorldZ()   {return mWorldZ;}
		double				getBodyX()		{return mBodyX;}
		double				getBodyY()		{return mBodyY;}
		double				getBodyZ()		{return mBodyZ;}

		bool				readConfig(const CConfigSection &configSection);
		void        randomize();
};

class CODEBodyIC: public CODELoggable
{
	protected:
		std::string		mBodyName;
		CODEMatrix3   mRotation;

		CODEInitializationExpression mXaxisX, mXaxisY, mXaxisZ, mYaxisX, mYaxisY, mYaxisZ;
		CODEInitializationExpression mAxisX, mAxisY, mAxisZ, mAngle;
		bool          mOrientationAsRotation;

	public:
		CODEBodyIC();

		void				setBodyName(const std::string& bodyName)	{mBodyName = bodyName;}
		const std::string&	getBodyName()	{return mBodyName;}
		CODEMatrix3&    getRotation() {return mRotation;} // For editing
		//const CODEMatrix3&	getRotation()	{return mRotation;}
		bool				readConfig(const CConfigSection &configSection);
    void        randomize(double r = 0);
};

// Object representing an external force, working on a
// particular point on a body of the object, having
// fixed magnitude and direction.
// The force vector is in the GLOBAL reference frame.
// The 'activation' point is in the BODY's reference frame.
class CODEExtForce: public CODELoggable
{
	protected:
		std::string		mBodyName;
		CODEBody*		mpBody;	// Will be resolved by init()
		CODEObject*		mpObject;
		double			mForceX, mForceY, mForceZ;
		double			mBodyX, mBodyY, mBodyZ;

	public:
		CODEExtForce(CODEObject *parentObject);

		const std::string&	getBodyName()	{return mBodyName;}
		bool				init();
		void				update();	// Apply the force to its body
		bool				readConfig(const CConfigSection &configSection);
		
		void			setBodyName(const std::string &name) { mBodyName = name; }
		void 			setForce(double forceX, double forceY, double forceZ)
		{
			mForceX = forceX;
			mForceY = forceY;
			mForceZ = forceZ;
		}
		
		void 			setPosition(double bodyX, double bodyY, double bodyZ)
		{
			mBodyX = bodyX;
			mBodyY = bodyY;
			mBodyZ = bodyZ;
		}
};

// **************************************************************************** //
// ********************************* objects ********************************** //
// **************************************************************************** //

// Forward declaration of CODESim
class CODESim;

// Encapsulation of a set of bodies, joints and geoms. Like a robot :)
// Each CODEObject has an mSpace member in which all geometries are inserted.
// A CODEObject itself cannot (yet) be inserted into another space (collision spaces are not yet treated hierarchically).
class CODEObject: public CODELoggable
{
	private:
		dHashSpace					mSpace;
    grl::Rand           mRand;

		// Bodies and geometries. An amount of 0 elements is allowed for both vectors.
		// CODEObject is OWNER of all bodies, geoms, joints and external forces
		CODESim*					mpSim;				// Pointer to the parent simulator. Used for world body and for ERPCFM-to-KD conversions, for joints.
		std::vector<CODEBody*>		mBodies;			// Bodies
		std::vector<CODEGeom*>		mGeoms;				// Geometries
		std::vector<CODEJoint*>		mJoints;			// Joints
		std::vector<CODEExtForce*>	mExternalForces;	// External forces
		std::vector<CODEBodyIC*>	mBodyICs;			// List of initial conditions of bodies
		void						clearBodies();
		void						clearGeoms();
		void						clearJoints();
		void						clearExternalForces();
		void						clearAll();
    void            genRandState(std::map<std::string, double> &jointMap);

	protected:
		std::string					mName;
		CODEObjectFixedPoint*		mFixedPoint;
		bool						mShouldDrawCoMs;
		bool						mShouldDrawBodies;
		bool						mShouldDrawJoints;
		bool						mShouldDrawGeoms;

		void						processFixedPoint();

	public:
		CODEObject(CODESim* pSim);
		virtual ~CODEObject();
		virtual bool		init(dWorld& world);
		virtual void		deinit();
		// Pass true to setInitialCondition() to ranzomize initial condition. Otherwise, the IC's are assumed to be set.
		void				setInitialCondition(bool randomize=true);

		const std::string&	name() const;


		void				drawCoMs(bool enabled);
		bool				shouldDrawCoMs();	// Indicates whether Centers of Mass should be drawn in visualization
		void				drawBodies(bool enabled);
		bool				shouldDrawBodies();	// Indicates whether bodies should be drawn in visualization
		void				drawJoints(bool enabled);
		bool				shouldDrawJoints();	// Indicates whether joints should be drawn in visualization
		void				drawGeoms(bool enabled);
		bool				shouldDrawGeoms();	// Indicates whether geoms should be drawn in visualization
		// Functions to add new bodies, geoms and joints
		// NOTE: CODEObject becomes OWNER of the added bodies, geoms and joints and will destroy them in deinit()
		void				addBody(CODEBody* pBody);
		void				addGeom(CODEGeom* pGeom);
		void				addJoint(CODEJoint* pJoint);
		void				addBodyIC(CODEBodyIC* pBodyIC);
		void				clearBodyICs();
		void				addExternalForce(CODEExtForce* pExtForce);

		CODEBody*			resolveBody(const std::string &bodyName);
		CODEJoint*			resolveJoint(const std::string &jointName);
		CODEGeom*			resolveGeom(const std::string &geomName);
		CODEBodyAnchor*		resolveAnchor(const std::string &bodyName, const std::string &anchorName);

		// dx, dy and dz are in the GLOBAL reference frame
		void				move(double dx, double dy, double dz);

		void				invalidateCollisions(bool invalid=true);

		const std::vector<CODEBody*>&		getBodies()			{return mBodies;}
		const std::vector<CODEJoint*>&		getJoints()			{return mJoints;}
		const std::vector<CODEGeom*>&		getGeoms()			{return mGeoms;}
		const std::vector<CODEExtForce*>&	getExternalForces()	{return mExternalForces;}
		dSpaceID							getSpaceID()		{return mSpace.id();}

		virtual bool		readConfig(const CConfigSection &configSection);

		// Helper function
		void				convertKDToERPCFM(double K, double D, double *ERP, double *CFM);
};

#endif
