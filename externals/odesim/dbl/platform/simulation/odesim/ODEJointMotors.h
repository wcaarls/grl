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

#ifndef ODEJOINTMOTORS_H_
#define ODEJOINTMOTORS_H_

#include <Configuration.h>
#include "ODELogging.h"

// Forward declaration of ODEJoint
class CODEJoint;

class CODEJointMotor: public CODELoggable
{
	protected:
		CODEJoint*		mpJoint;
		int				mAxisIndex;	// Determines which joint axis this motor actuates (joints can have several axes)

	public:
		CODEJointMotor(CODEJoint* joint);
		virtual ~CODEJointMotor() { }

		virtual void	setTorque(double torque)	{}
		virtual double  getTorque()                 {return 0;}
		virtual void	setForce(double force)		{}
                virtual double  getForce()                  {return 0;}
		virtual void	setVoltage(double voltage)	{}	// WARNING: This function does not have to be implemented
		virtual double  getVoltage()                {return 0;}

		virtual void	setInitialCondition()		{}	// Updates the internal state of the motor according to the initial state of the connected bodies
		virtual void	update(double stepTime)	{}
		virtual bool	readConfig(const CConfigSection &configSection);
};

// Simple motor that outputs the torque that you want.
// Extra options: linear damping (proportional to the joint velocity)
class CODETorqueMotor: public CODEJointMotor
{
	protected:
		double	mTorque;
		double	mLinearDamping;	// Damping proportional to the joint's angular velocity

	public:
		CODETorqueMotor(CODEJoint* joint);
		virtual ~CODETorqueMotor() { }

		void	setTorque(double torque);
		double  getTorque()			{return mTorque;}
		double	getLinearDamping()	{return mLinearDamping;}
		void	update(double stepTime);
		bool	readConfig(const CConfigSection &configSection);
};

// Simple motor that outputs the force that you want.
// Extra options: linear damping (proportional to the joint velocity)
class CODEForceMotor: public CODEJointMotor
{
	protected:
		double	mForce;
		double	mLinearDamping;	// Damping proportional to the joint's angular velocity

	public:
		CODEForceMotor(CODEJoint* joint);
		virtual ~CODEForceMotor() { }

		void	setForce(double torque);
		double  getForce()			{return mForce;}
		double	getLinearDamping()	{return mLinearDamping;}
		void	update(double stepTime);
		bool	readConfig(const CConfigSection &configSection);
};

// Current-controlled servo motor
class CODEServoMotor: public CODEJointMotor
{
	private:	// debug
		FILE	*mLogFile;
	protected:
		// The core control parameter of this servo motor model is the DC motor voltage
		double	mVoltage;

		// We model the armature of the motor as a virtual disc with effective inertia only,
		// placed on the floor, only being able to rotate around a single axis,
		// connected to the joint by a spring-damper combination (K, D).
		double	mEffectiveInertia;	// This equals I_motorarmature*R^2
		double	mVDiscPos;		// The position of the virtual motor disc
		double	mVDiscVel;		// The angular velocity of the virtual motor disc
		double	mVDiscCtrlK;	// Stiffness of the coupling controller between the virtual motor disc and the joint
		double	mVDiscCtrlD;	// Damping of the coupling controller between the virtual motor disc and the joint

		// additional motor values
		double	mTorqueConstant;
		double	mTerminalResistance;
		double	mGearboxRatio;
		double	mGearboxEfficiency;	// Number between 0 and 1
		double	mSupplyVoltage;

		int		mPrintMod;	// DEBUG var

		double	updateVDisc(double stepTime);	// Returns the torque that was applied to the virtual motor disc

	public:
		CODEServoMotor(CODEJoint* joint);
		virtual ~CODEServoMotor();

		void	setVoltage(double voltage);
		double  getVoltage() {return mVoltage;}
		void	setTorque(double torque);
		void	update(double stepTime);
		void	setInitialCondition();
		bool	readConfig(const CConfigSection &configSection);
};

// This class was originally created to implement the 'endless turn mode' of the Dynamixel.
// However, this turns out to be voltage control, and this functionality is added to CODEServoMotor.
class CODEDynamixel: public CODEServoMotor
{
	public:
		CODEDynamixel(CODEJoint* joint);
		virtual ~CODEDynamixel();
};

// Global class factory function //
CODEJointMotor* gODECreateJointMotor(const std::string& motorTypeStr, CODEJoint *parentJoint);


#endif /* ODEJOINTMOTORS_H_ */
