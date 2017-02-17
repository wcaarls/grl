/*
 * XM430_210_Specs.h
 *
 *  Created on: Jan 27, 2017
 *      Author: Ivan Koryakovskiy
 */

#ifndef DYNAMIXEL_XM430_210_SPECS_H_
#define DYNAMIXEL_XM430_210_SPECS_H_

#define DXL_XM430_210_PWM_TO_VOLTAGE        (LEO_SUPPLY_VOLTAGE/885.0)        // [V/unit]
#define DXL_XM430_210_VOLTAGE_UNIT          0.1                               // [V/unit]
#define DXL_XM430_210_CURRENT_UNIT          0.00269                           // [A/unit]

#define DXL_XM430_210_CURRENT_LIMIT         1193
#define DXL_XM430_210_CURRENT_UNIT_MAX      (DXL_XM430_210_MAX_CURRENT/ DXL_XM430_210_CURRENT_UNIT)  // = 964.96; maximum value of a current register in dynamixel


#define DXL_XM430_210_NUM_POSITIONS         4096                    // 12-bit encoder -> 4096 positions
#define DXL_XM430_210_MAX_POSITION          4095                    // 12-bit = 0 - 4095 values

// speed is 10-bit with approximately 0.229 RPM / bit (manual: 'Goal Velocity')
// bitSpeed * 0.229 -> speed in RPM
// (2*M_PI/60) = conversion: RPM -> rad/s
// = 0.011938052
#define DXL_XM430_210_SPEED_TO_RAD_S        0.104716667             // = 2*M_PI/60
#define DXL_XM430_210_STEPS_TO_RAD          0.001533980787886       // = 2*M_PI/4096 (4096 positions in 2*pi rad)
//  #define DXL_XM430_210_TORQUE_TO_RATIO      0.00097751710654936461  // = (1.0/1023.0) (torque is 10-bit)

// By quadratic interpolation, at 13.8 V we get stall torque of 2.841132
//  #define DXL_XM430_210_MAX_TORQUE_PER_VOLT  (2.841132/13.8)  // Max torques in [Nm]

// Not loaded speed @ 13.8 [V]: 89.2277992278 rpm (interpolated with datasheet values) = 9.343637739 rad s^-1
//  #define DXL_XM430_210_RPM_PER_VOLT          (89.2277992278/LEO_SUPPLY_VOLTAGE)  // [rpm]
#define DXL_XM430_210_VELOCITY_UNIT         (0.229*DXL_XM430_210_SPEED_TO_RAD_S)    // [rad/s/unit] = 0.023980117
#define DXL_XM430_210_VELOCITY_LIMIT        1023  // max velocity = 24.531659428 rad/s
#define DXL_XM430_210_MAX_VELOCITY          (DXL_XM430_210_VELOCITY_UNIT*DXL_XM430_210_VELOCITY_LIMIT)

// Motor coefficients
#define DXL_XM430_210_GEARBOX_RATIO         212.6                   // from specifications:  212.6 : 1 reduction

// Stall torque @ 13.8 [V]: 3.49054054054 [Nm] (interpolated with datasheet values)
#define DXL_XM430_210_MAX_TORQUE            3.49054054054

// Stall current @ 13.8 [V]: 2.59575289575 [A] (interpolated with datasheet values)
#define DXL_XM430_210_MAX_CURRENT           2.59575289575

// Coeffecient which says how much the XM430 motor is stronger then RX28
#define XM430_VS_RX28_COEFF                 0.8

// Torque constant of the whole dynamixel
// ( stallTorque / stallCurrent ) / gearRatio
#define DXL_XM430_210_TORQUE_CONST          (DXL_XM430_210_MAX_TORQUE/DXL_XM430_210_MAX_CURRENT) // = 1.344712182 [Nm/A]

// ... and torque constant of the Maxon motor inside
#define MAXON_TORQUE_CONST                  (DXL_XM430_210_TORQUE_CONST/DXL_XM430_210_GEARBOX_RATIO) // = 0.006325081 [Nm/A]

// Temperature coefficients
#define DXL_XM430_210_COPPER_COEF           0.004041                // @20 deg C, according to: http://www.cirris.com/learning-center/general-testing/special-topics/177-temperature-coefficient-of-copper

// Was 0.0041, which worked well with motorCoeff=-0.002 for half compensation. And tempRef = 20deg.
#define DXL_XM430_210_MAGNET_COEF           (-0.00)  // According to Maxon, this is -0.0011. (Was -0.002, which worked well for half compensation (not logical! since back-emf is ignored))

#define DXL_XM430_210_MAX_RAD_S_SPEED       (DXL_XM430_210_MAX_POSITION*DXL_XM430_210_SPEED_TO_RAD_S)
#define DXL_XM430_210_MAX_RAD_ANGLE         (DXL_XM430_210_MAX_POSITION*DXL_XM430_210_STEPS_TO_RAD)

#endif /* DYNAMIXEL_XM430_210_SPECS_H_ */
