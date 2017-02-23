/*
 * DynamixelSpecs.h
 *
 *  Created on: Jun 19, 2009
 *      Author: Erik Schuitema
 */

#ifndef DYNAMIXEL_RX28_SPECS_H_
#define DYNAMIXEL_RX28_SPECS_H_

#define DXL_RX28_VOLTAGE_UNIT         0.1

#define DXL_RX28_NUM_POSITIONS        1024
#define DXL_RX28_MAX_POSITION         1023
#define DXL_RX28_MAX_VELOCITY         90 // <- Just guessed! needs to be updated
#define DXL_RX28_SPEED_TO_RAD_S       0.01163552834662886385  // = (M_PI/270.0)  // Multiply speed with 0.111 (~=1/9) to get RPM. Multiply RPM with (2pi/60) for rad/s.
#define DXL_RX28_STEPS_TO_RAD         0.00511826760115639172   // = ((300.0/360.0)*2.0*M_PI/1023.0)
#define DXL_RX28_TORQUE_TO_RATIO      0.00097751710654936461  // = (1.0/1023.0)

#define DXL_RX28_MAX_TORQUE_PER_VOLT  (3.3/14)  // Max torques in [Nm]
#define DXL_RX28_MAX_SPEED_PER_VOLT   (40.75*DXL_RX28_SPEED_TO_RAD_S)  // Max speeds in rad/s

// Motor coefficients
#define DXL_RX28_TORQUE_CONST         0.00992
#define DXL_RX28_GEARBOX_RATIO        193.0

// Temperature coefficients
#define DXL_RX28_COPPER_COEF          0.004  // Was 0.0041, which worked well with motorCoeff=-0.002 for half compensation. And tempRef = 20deg.
#define DXL_RX28_MAGNET_COEF          0.000  // According to Maxon, this is -0.0011. (Was -0.002, which worked well for half compensation (not logical! since back-emf is ignored))

#define DXL_RX28_INITIAL_PUNCH        32

#define DXL_RX28_MAX_RAD_S_SPEED       (DXL_RX28_MAX_POSITION*DXL_RX28_SPEED_TO_RAD_S)
#define DXL_RX28_MAX_RAD_ANGLE         (DXL_RX28_MAX_POSITION*DXL_RX28_STEPS_TO_RAD)

#endif /* DYNAMIXEL_RX28_SPECS_H_ */
