/*
 * DynamixelSpecs.h
 *
 *  Created on: Jun 19, 2009
 *      Author: Erik Schuitema
 */

#ifndef DYNAMIXELSPECS_H_
#define DYNAMIXELSPECS_H_

// Conversions
#define DXL_RX28_NUM_POSITIONS			1024
#define DXL_RX28_MAX_POSITION			1023
#define DXL_RX28_SPEED_TO_RAD_S			0.01163552834662886385  // = (M_PI/270.0)	// Multiply speed with 0.111 (~=1/9) to get RPM. Multiply RPM with (2pi/60) for rad/s.
#define DXL_RX28_STEPS_TO_RAD			0.00511826760115639172 	// = ((300.0/360.0)*2.0*M_PI/1023.0)

#define DXL_RX64_SPEED_TO_RAD_S			DXL_RX28_SPEED_TO_RAD_S
#define DXL_RX64_STEPS_TO_RAD			DXL_RX28_STEPS_TO_RAD

#define DXL_RX28_TORQUE_TO_RATIO		0.00097751710654936461	// = (1.0/1023.0)

#define DXL_VOLTAGE_TO_VOLT				0.1


// Max torques in [Nm]
#define DXL_RX28_MAX_TORQUE_PER_VOLT	(3.3/14)
#define DXL_RX64_MAX_TORQUE_PER_VOLT	(0.4291)
// Max speeds
#define DXL_RX28_MAX_SPEED_PER_VOLT		(40.75*DXL_RX28_SPEED_TO_RAD_S)	// In rad/s
#define DXL_RX64_MAX_SPEED_PER_VOLT		(31.78*DXL_RX64_SPEED_TO_RAD_S)	// In rad/s. Yes, this is lower than for the RX-28.

// Motor coefficients
#define DXL_RX28_TORQUE_CONST			0.00992
#define DXL_RX28_GEARBOX_RATIO			193.0
#define DXL_RX28_RESISTANCE           8.6

// Temperature coefficients
#define DXL_RX28_COPPER_COEF			0.004	// Was 0.0041, which worked well with motorCoeff=-0.002 for half compensation. And tempRef = 20deg.
#define DXL_RX28_MAGNET_COEF			(-0.00)	// According to Maxon, this is -0.0011. (Was -0.002, which worked well for half compensation (not logical! since back-emf is ignored))


#endif /* DYNAMIXELSPECS_H_ */
