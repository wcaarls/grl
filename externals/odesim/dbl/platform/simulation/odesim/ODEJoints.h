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

#ifndef ODEJOINTS_H_
#define ODEJOINTS_H_

#include <ode/ode.h>
#include <Configuration.h>
#include "ODEBodies.h"
#include "ODEJointMotors.h"
#include "ODELogging.h"
//#include <vector>

// **************************************************************************** //
// ********************************* joints *********************************** //
// **************************************************************************** //

// Forward declaration
class CODEObject;

class CODEJoint: public CODELoggable
{
	private:
		// intentionally undefined, don't use these
		CODEJoint(const CODEJoint &);
		void operator= (const CODEJoint &);

		bool					assertAnchors();

	protected:
		dJointID				mID;
		std::string				mName;
		CODEObject*				mpObject;	// Parent object. Used to resolve body anchors
		CODEBodyAnchor*			mpBodyAnchors[2];

		virtual void			create(dWorld& world, dJointGroupID groupID)=0;	// Abstract; you must implement it

	public:
		CODEJoint(CODEObject *parentObject);
		virtual ~CODEJoint();

		const std::string&		name() const		{ return mName; }
		dJointID				id() const			{ return mID; }
		operator				dJointID() const	{ return mID; }

		// connectBodies(): moves (but does not rotate) the two attached bodies in such a way that the joint anchor points overlap again
		// It also reports the bodies to each other as being connected bodies
		// It does NOT attach the ODE bodies to the ODE joint (so no call to dJointAttach)
		void					connectBodies();	// Call this inside your own overridden init()

		virtual void			setParams()	{}	// Set additional params like anchors, axes, etc.
		virtual bool			readConfig(const CConfigSection &configSection);
		virtual bool			init(dWorld& world, dJointGroupID groupID=0);
		virtual void			deinit()	{}

		// Returns number of POSSIBLE motors in a joint. It doesn't mean there are any motors present (so check for NULL from getMotor()).
		virtual unsigned int	getNumMotors()				{return 0;}
		// Returns motor, if any. Index is used to specify a joint axis (some joints have more than 1).
		virtual CODEJointMotor*	getMotor(int motorIndex)	{return NULL;}

		// Angle and anglerate may be defined for more degrees of freedom within one joint (e.g. Hinge2 and Universal)
		// Angle should always be in the range [-PI, PI].
		virtual double			getAngle(int axisIndex=0) const		{return 0;}
		virtual double			getAngleRate(int axisIndex=0) const	{return 0;}

		// Position and positionrate of a linear joint.
		virtual double			getPosition() const					{return 0;}
		virtual double			getPositionRate() const				{return 0;}


		// Torque or force may not be implemented on all joints!
		virtual void			addTorque(double torque, int axisIndex=0)	{}	// Only valid for the next simulation SUB-step!
		virtual void			addForce(double force)						{}	// Only valid for the next simulation SUB-step!

		dJointType				getType() const		{return dJointGetType(mID);}
		std::string				getTypeName() const;
		void					enable()			{dJointEnable(mID);}
		void					disable()			{dJointDisable(mID);}
		bool					isEnabled() const	{return dJointIsEnabled(mID) != 0;}

		// Helper function; depends on simulator step time
		void					convertKDToERPCFM(double K, double D, double *ERP, double *CFM);

};

class CODEBallJoint: public CODEJoint
{
	protected:
		void		create(dWorld& world, dJointGroupID groupID);

	public:
		CODEBallJoint(CODEObject *parentObject): CODEJoint(parentObject)	{}
		void		setParams();
};

class CODEHingeJoint: public CODEJoint
{
	protected:
		CODEJointMotor	*mMotor;
		double			mAxisX, mAxisY, mAxisZ;
		double			mStopERP, mStopCFM;
		double			mDryFriction;
		double			mLowerLimit, mUpperLimit;
		void			create(dWorld& world, dJointGroupID groupID);

	public:
		CODEHingeJoint(CODEObject *parentObject);
		virtual ~CODEHingeJoint();

		bool			init(dWorld& world, dJointGroupID groupID=0);

		void			setParams();
		double			getAngle(int axisIndex=0) const;
		double			getAngleRate(int axisIndex=0) const;
		double			getLowerLimit() const	{return mLowerLimit;}
		double			getUpperLimit() const	{return mUpperLimit;}
		void			addTorque(double torque, int axisIndex=0);

		void			setMotor(CODEJointMotor* motor);	// CODEHingeJoint becomes OWNER of the motor
		unsigned int	getNumMotors();
		CODEJointMotor*	getMotor(int motorIndex=0);

		bool			readConfig(const CConfigSection &configSection);

};

class CODESliderJoint: public CODEJoint
{
	protected:
		CODEJointMotor	*mMotor;
		double			mAxisX, mAxisY, mAxisZ;
		double			mStopERP, mStopCFM;
		double			mDryFriction;
		double			mLowerLimit, mUpperLimit;
		void			create(dWorld& world, dJointGroupID groupID);

	public:
		CODESliderJoint(CODEObject *parentObject);
		virtual ~CODESliderJoint();

		bool			init(dWorld& world, dJointGroupID groupID=0);

		void			setParams();
		double			getPosition() const;
		double			getPositionRate() const;
		double			getLowerLimit() const	{return mLowerLimit;}
		double			getUpperLimit() const	{return mUpperLimit;}
		void			addForce(double force);

		void			setMotor(CODEJointMotor* motor);	// CODEHingeJoint becomes OWNER of the motor
		unsigned int	getNumMotors();
		CODEJointMotor*	getMotor(int motorIndex=0);

		bool			readConfig(const CConfigSection &configSection);
};

class CODEUniversalJoint: public CODEJoint
{
	protected:
		double		mAxis1X, mAxis1Y, mAxis1Z;
		double		mAxis2X, mAxis2Y, mAxis2Z;
		void		create(dWorld& world, dJointGroupID groupID);

	public:
		CODEUniversalJoint(CODEObject *parentObject): CODEJoint(parentObject)	{}
		void		setParams();
		bool		readConfig(const CConfigSection &configSection);
};

class CODEFixedJoint: public CODEJoint
{
	protected:
		void		create(dWorld& world, dJointGroupID groupID);

	public:
		CODEFixedJoint(CODEObject *parentObject): CODEJoint(parentObject)	{}
		void		setParams();
};

// Global class factory function //
CODEJoint* gODECreateJoint(const std::string& jointTypeStr, CODEObject *parentObject);

/*

class CODEHinge2Joint: public dHinge2Joint, public CODEJoint
{

};

class CODEPRJoint: public dPRJoint, public CODEJoint
{

};

class CODEPUJoint: public dPUJoint, public CODEJoint
{

};

class CODEPistonJoint: public dPistonJoint, public CODEJoint
{

};

class CODEJointAMotor: public dAMotorJoint, public CODEJoint
{

};

class CODEJointLMotor: public dLMotorJoint, public CODEJoint
{

};
*/
// Contact joint will be a separate thing

#endif /* ODEJOINTS_H_ */
