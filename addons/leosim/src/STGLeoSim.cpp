/*
 * STGLeoSim.cpp
 *
 *  Created on: Apr 9, 2009
 *      Author: Erik Schuitema
 */

#include "STGLeoSim.h"

// ******************************************************************************* //
// ********************************* CSTGLeoSim ********************************** //
// ******************************************************************************* //

double CSTGLeoSim::getJointVoltage(int jointIndex)
{
  return mMotorJointVoltages[jointIndex];
}

void CSTGLeoSim::setJointVoltage(int jointIndex, double voltage)
{
	if (jointIndex < ljNumDynamixels)
	{
		// Don't allow voltages larger than the maximum. In the real robot, the Dynamixel actuation functions limit the voltage.
		double voltageLimit = getJointMaxVoltage(jointIndex);
		double clippedVoltage = std::max(std::min(voltage, voltageLimit), -voltageLimit);
    mMotorJointVoltages[jointIndex] = clippedVoltage;
	}
}

double CSTGLeoSim::getJointMaxVoltage(int jointIndex)
{
	// Leo runs on a fixed voltage power supply, and all motors are RX-28's
	// Voltages are limited below the supply voltage due to temperature compensation (see macro definition)
	if (jointIndex < ljNumDynamixels)
		return LEO_MAX_DXL_VOLTAGE;
	else
    return 0;
}

