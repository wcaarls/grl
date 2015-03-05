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

#include "ODEJoints.h"
#include "ODEJointMotors.h"

CODEJointMotor::CODEJointMotor(CODEJoint* joint):
	mpJoint(joint),
	mAxisIndex(0)
{
}

bool CODEJointMotor::readConfig(const CConfigSection &configSection)
{
	bool configresult = true;

	// Not necessary to define axis index; in that case, 0 is assumed (see constructor)
	configSection.get("axis", &mAxisIndex);

	return configresult;
}

CODETorqueMotor::CODETorqueMotor(CODEJoint* joint):
	CODEJointMotor(joint),
	mTorque(0.0),
	mLinearDamping(0.0)
{

}

void CODETorqueMotor::setTorque(double torque)
{
	mTorque = torque;
}

void CODETorqueMotor::update(double stepTime)
{
	double finalTorque = mTorque - mLinearDamping*mpJoint->getAngleRate(mAxisIndex);
	mpJoint->addTorque(finalTorque, mAxisIndex);
}

bool CODETorqueMotor::readConfig(const CConfigSection &configSection)
{
	bool configresult = true;

	configresult &= CODEJointMotor::readConfig(configSection);

	// Not necessary to define damping; in that case, 0 is assumed (see constructor)
	configSection.get("lineardamping", &mLinearDamping);

	return configresult;
}

CODEForceMotor::CODEForceMotor(CODEJoint* joint):
	CODEJointMotor(joint),
	mForce(0.0),
	mLinearDamping(0.0)
{

}

void CODEForceMotor::setForce(double force)
{
	mForce = force;
}

void CODEForceMotor::update(double stepTime)
{
	double finalForce = mForce - mLinearDamping*mpJoint->getPositionRate();
	mpJoint->addForce(finalForce);
}

bool CODEForceMotor::readConfig(const CConfigSection &configSection)
{
	bool configresult = true;

	configresult &= CODEJointMotor::readConfig(configSection);

	// Not necessary to define damping; in that case, 0 is assumed (see constructor)
	configSection.get("lineardamping", &mLinearDamping);

	return configresult;
}

CODEServoMotor::CODEServoMotor(CODEJoint* joint):
	CODEJointMotor(joint),
	mVoltage(0.0),
	mEffectiveInertia(0.0),
	mVDiscPos(0.0),
	mVDiscVel(0.0),
	mVDiscCtrlK(0.0),
	mVDiscCtrlD(0.0),
	mTorqueConstant(0.00992),
	mTerminalResistance(9.5),
	mGearboxRatio(193.0),
	mGearboxEfficiency(1.0),
	mSupplyVoltage(13.6),
	mPrintMod(0)
{
}

CODEServoMotor::~CODEServoMotor()
{
}

void CODEServoMotor::setTorque(double torque)
{
	// Assumption: the servo applies current-control
	// Calculate voltage from desired torque. This does not take the gear box efficiency into account!
	mVoltage	= (torque*mTerminalResistance/(mGearboxRatio*mTorqueConstant)) + mGearboxRatio*mTorqueConstant*mpJoint->getAngleRate(mAxisIndex);
	// Clipping of the voltage is done inside update()
}

void CODEServoMotor::setVoltage(double voltage)
{
	mVoltage	= voltage;
}

void CODEServoMotor::setInitialCondition()
{
	mVDiscPos	= mpJoint->getAngle(mAxisIndex);
	mVDiscVel	= mpJoint->getAngleRate(mAxisIndex);
}

/*// Backup
void CODEServoMotor::update(double stepTime)
{
	double finalTorque	= mTorque;
	finalTorque -= mLinearDamping*mpJoint->getAngleRate(mAxisIndex);
	// Update virtual motor disc model

	// no-ERP way:
	double posError				= (mpJoint->getAngle(mAxisIndex) - mMotorPos);
	double velError				= (mpJoint->getAngleRate(mAxisIndex) - mMotorVel);
	double motorDiscCtrlTorque	= mMotorCtrlK*posError + mMotorCtrlD*velError;
	double motorDiscWork		= motorDiscCtrlTorque*mMotorVel;
	mMotorVel += stepTime*motorDiscCtrlTorque/mMotorInertia;
	mMotorPos += stepTime*mMotorVel;

	// ERP way:
//	double motorERP = 0.2;	// Reduce the velocity error every step by 80%
//	double motorCFM = 0.01;	// Allow "1% deviation"
//	double motorDiscCtrlTorque = motorERP*((mMotorInertia/stepTime)/stepTime)*(mpJoint->getAngle(mAxisIndex)-mMotorPos); //(1.0-motorCFM)*( (mMotorInertia/stepTime)*(mpJoint->getAngleRate(mAxisIndex)-mMotorVel) );
//	mMotorVel += stepTime*motorDiscCtrlTorque/mMotorInertia;
//	mMotorPos += stepTime*mMotorVel;

	printmod++;
	if (printmod % 100 == 0)
	{
		//printf("[DEBUG] Joint %s: x_motor: %.4f (x_joint %.4f), v_motor: %.4f (v_joint %.4f)\n", mpJoint->name().c_str(), mMotorPos, mpJoint->getAngle(mAxisIndex), mMotorVel, mpJoint->getAngleRate(mAxisIndex));
	}
	//fprintf(mLogFile, "%.15f\t%.15f\t%.15f\t%.15f\t%.15f\t%.15f\t%.15f\t%.15f\t\n", mpJoint->getAngle(mAxisIndex), mMotorPos, mpJoint->getAngleRate(mAxisIndex), mMotorVel, posError, velError, motorDiscCtrlTorque, motorDiscWork);
	finalTorque -= motorDiscCtrlTorque;	// Equal but opposite torque acts on the joint!

//	double maxTorque	= 3.0;//mMaxTorque*cos(0.3*M_PI*fabs(mpJoint->getAngleRate(mAxisIndex))/mMaxSpeed);
//	if (finalTorque > maxTorque)
//		finalTorque = maxTorque;
//	else if (finalTorque < -maxTorque)
//		finalTorque = -maxTorque;

	mpJoint->addTorque(finalTorque, mAxisIndex);
	//printf("[DEBUG] Joint  %s: turned torque %.10f into %.10f; angle rate %.10f\n", mpJoint->name().c_str(), mTorque, finalTorque, mpJoint->getAngleRate(mAxisIndex));
}
*/

double CODEServoMotor::updateVDisc(double stepTime)
{
	// Update virtual motor disc model
	double posError			= (mpJoint->getAngle(mAxisIndex) - mVDiscPos);
	double velError			= (mpJoint->getAngleRate(mAxisIndex) - mVDiscVel);
	double VDiscCtrlTorque	= mVDiscCtrlK*posError + mVDiscCtrlD*velError;
	//double VDiscWork		= motorDiscCtrlTorque*mMotorVel;
	mVDiscVel += stepTime*VDiscCtrlTorque/mEffectiveInertia;
	mVDiscPos += stepTime*mVDiscVel;
	return VDiscCtrlTorque;
}

void CODEServoMotor::update(double stepTime)
{
	// DC motor model:
	// P_in = P_out + P_loss,electric
	// U*I = T*omega + I^2*R_motor
	// T = k*I
	// U = k*omega + I*R_motor
	// I = (U - k*omega) / R_motor
	// T_max_motor = k * (U - k*omega) / R_motor
	// Gearbox model: dry friction, independent of load (unrealistic!!) = T_loss,gb
	//

	// Clip desired voltage to supply voltage
	double clippedVoltage = mVoltage;
	if (clippedVoltage > mSupplyVoltage)
		clippedVoltage = mSupplyVoltage;
	if (clippedVoltage < -mSupplyVoltage)
		clippedVoltage = -mSupplyVoltage;

	// Calculate torque from voltage
	double torque = mGearboxRatio*mTorqueConstant*(clippedVoltage - mTorqueConstant*mGearboxRatio*mpJoint->getAngleRate(mAxisIndex))/mTerminalResistance;

	// Update virtual motor disc model if the armature has inertia
	if (mEffectiveInertia > 0)
	{
		double VDiscCtrlTorque	= updateVDisc(stepTime);
		torque -= VDiscCtrlTorque;	// Equal but opposite torque acts on the joint
	}

	// Gearbox inefficiency
	torque *= mGearboxEfficiency;

	mpJoint->addTorque(torque, mAxisIndex);

	// Report to screen if desired
	/*
	if (mPrintMod % 105 == 0)
	{
		printf("[DEBUG] Joint %s: x_motor: %.4f (x_joint %.4f), v_motor: %.4f (v_joint %.4f)\n", mpJoint->name().c_str(), mVDiscPos, mpJoint->getAngle(mAxisIndex), mVDiscVel, mpJoint->getAngleRate(mAxisIndex));
	}
	mPrintMod++;
	*/
}

bool CODEServoMotor::readConfig(const CConfigSection &configSection)
{
	bool configresult = true;

	configresult &= CODEJointMotor::readConfig(configSection);

	double armatureInertia=0;
	configresult &= mLogAssert(configSection.get("torqueconstant", &mTorqueConstant));
	configresult &= mLogAssert(configSection.get("terminalresistance", &mTerminalResistance));
	configresult &= mLogAssert(configSection.get("gearboxratio", &mGearboxRatio));
	configresult &= mLogAssert(configSection.get("gearboxefficiency", &mGearboxEfficiency));
	configresult &= mLogAssert(configSection.get("supplyvoltage", &mSupplyVoltage));
	configSection.get("armatureinertia", &armatureInertia);
	mEffectiveInertia = armatureInertia*mGearboxRatio*mGearboxRatio;
	if (mEffectiveInertia > 0)
	{
		configresult &= mLogAssert(configSection.get("trackerK", &mVDiscCtrlK));
		configresult &= mLogAssert(configSection.get("trackerD", &mVDiscCtrlD));
	}

	return configresult;
}

CODEDynamixel::CODEDynamixel(CODEJoint* joint):
	CODEServoMotor(joint)
{

}

CODEDynamixel::~CODEDynamixel()
{

}

//void CODEDynamixel::setTorque(double torque)
//{
//	// Old plan for 'torque control', which it wasn't: apply motor voltage for which voltage/mSupplyVoltage == mTorque/stallTorque;
//	// Calculate maximum torque of this motor
//	double stallTorque = mGearboxRatio*mTorqueConstant*mSupplyVoltage/mTerminalResistance;
//	// Clip desired torque to stalltorque
//	if (mTorque > stallTorque)
//		mTorque = stallTorque;
//	if (mTorque < -stallTorque)
//		mTorque = -stallTorque;
//
//	// Apply voltage for which voltage/mSupplyVoltage == mTorque/stallTorque;
//	double U = mSupplyVoltage*mTorque/stallTorque;
//	double torque = mGearboxRatio*mTorqueConstant*(U - mTorqueConstant*mGearboxRatio*mpJoint->getAngleRate(mAxisIndex))/mTerminalResistance;
//}
//

// Global class factory function //
CODEJointMotor* gODECreateJointMotor(const std::string& motorTypeStr, CODEJoint *parentJoint)
{
	CODEJointMotor* newMotor = NULL;

	if (motorTypeStr == "torque")
		newMotor = new CODETorqueMotor(parentJoint);
	else if (motorTypeStr == "force")
		newMotor = new CODEForceMotor(parentJoint);
	else if (motorTypeStr == "servo")
		newMotor = new CODEServoMotor(parentJoint);
	else if (motorTypeStr == "dynamixel")
		newMotor = new CODEDynamixel(parentJoint);

	return newMotor;
}
