/*
 * DynamixelSpecs.h
 *
 *  Created on: Jun 19, 2009
 *      Author: Erik Schuitema
 */

#ifndef DYNAMIXELSPECS_H_
#define DYNAMIXELSPECS_H_

// Select the proper Dynamixel motor here
//#define RX28
//#define MX28
#define XM430_210

#define LEO_SUPPLY_VOLTAGE              13.8  // [V]

#ifdef RX28
  #include "RX28/RX28_Specs.h"

  #define DXL_MAX_TORQUE_PER_VOLT       DXL_RX28_MAX_TORQUE_PER_VOLT
  #define DXL_MAX_SPEED_PER_VOLT        DXL_RX28_MAX_SPEED_PER_VOLT

  #define DXL_MAX_RAD_S_SPEED           DXL_RX28_MAX_RAD_S_SPEED
  #define DXL_MAX_RAD_ANGLE             DXL_RX28_MAX_RAD_ANGLE

  // Motor coefficients
  #define DXL_TORQUE_CONST              DXL_RX28_TORQUE_CONST
  #define DXL_GEARBOX_RATIO             DXL_RX28_GEARBOX_RATIO
  #define DXL_RESISTANCE                8.6

  // Temperature coefficients
  #define DXL_COPPER_COEF               DXL_RX28_COPPER_COEF
  #define DXL_MAGNET_COEF               DXL_RX28_MAGNET_COEF

  #define DXL_NUM_POSITIONS             DXL_RX28_NUM_POSITIONS
  #define DXL_MAX_POSITION              DXL_RX28_MAX_POSITION
  #define DXL_MAX_VELOCITY              DXL_RX28_MAX_VELOCITY
  #define DXL_SPEED_TO_RAD_S            DXL_RX28_SPEED_TO_RAD_S
  #define DXL_STEPS_TO_RAD              DXL_RX28_STEPS_TO_RAD
  #define DXL_TORQUE_TO_RATIO           DXL_RX28_TORQUE_TO_RATIO
#endif

#ifdef MX28
  #include "MX28/MX28_Specs.h"

  #define DXL_MAX_TORQUE_PER_VOLT       DXL_MX28_MAX_TORQUE_PER_VOLT
  #define DXL_MAX_SPEED_PER_VOLT        DXL_MX28_MAX_SPEED_PER_VOLT

  #define DXL_MAX_RAD_S_SPEED           DXL_MX28_MAX_RAD_S_SPEED
  #define DXL_MAX_RAD_ANGLE             DXL_MX28_MAX_RAD_ANGLE

  // Motor coefficients
  #define DXL_TORQUE_CONST              DXL_MX28_TORQUE_CONST
  #define DXL_GEARBOX_RATIO             DXL_MX28_GEARBOX_RATIO
  #define DXL_RESISTANCE                8.6

  // Temperature coefficients
  #define DXL_COPPER_COEF               DXL_MX28_COPPER_COEF
  #define DXL_MAGNET_COEF               DXL_MX28_MAGNET_COEF

  #define DXL_NUM_POSITIONS             DXL_MX28_NUM_POSITIONS
  #define DXL_MAX_POSITION              DXL_MX28_MAX_POSITION
  #define DXL_MAX_VELOCITY              DXL_MX28_MAX_VELOCITY
  #define DXL_SPEED_TO_RAD_S            DXL_MX28_SPEED_TO_RAD_S
  #define DXL_STEPS_TO_RAD              DXL_MX28_STEPS_TO_RAD
  #define DXL_TORQUE_TO_RATIO           DXL_MX28_TORQUE_TO_RATIO
#endif

#ifdef XM430_210
  #include "XM430/XM430_210_Specs.h"

  #define DXL_MAX_RAD_S_SPEED                 DXL_XM430_210_MAX_RAD_S_SPEED
  #define DXL_MAX_RAD_ANGLE                   DXL_XM430_210_MAX_RAD_ANGLE

  #define DXL_MAX_POSITION                    DXL_XM430_210_MAX_POSITION
  #define DXL_MAX_VELOCITY                    DXL_XM430_210_MAX_VELOCITY

  // Motor coefficients
  #define DXL_TORQUE_CONST                    MAXON_TORQUE_CONST
  #define DXL_GEARBOX_RATIO                   DXL_XM430_210_GEARBOX_RATIO
  #define DXL_RESISTANCE                      4.6

  // Temperature coefficients
  #define DXL_COPPER_COEF                     DXL_XM430_210_COPPER_COEF
  #define DXL_MAGNET_COEF                     DXL_XM430_210_MAGNET_COEF

  #define DXL_NUM_POSITIONS                   DXL_XM430_210_NUM_POSITIONS
  #define DXL_MAX_POSITION                    DXL_XM430_210_MAX_POSITION
  #define DXL_SPEED_TO_RAD_S                  DXL_XM430_210_SPEED_TO_RAD_S
  #define DXL_STEPS_TO_RAD                    DXL_XM430_210_STEPS_TO_RAD
#endif

// ** Temperature compensation used in voltage/PWM control mode ** //

// Temperature compensation defines
#define LEO_DXL_REF_TEMP                25.0    // Use 25.0 with copperfact=0.004 and magnetfact=0.0
#define LEO_DXL_MAX_TEMP                75.0
#define LEO_DXL_MAX_TEMP_DIFF           (LEO_DXL_MAX_TEMP - LEO_DXL_REF_TEMP)
#define LEO_DXL_MAX_BREAKING_VEL        4.0        // Maximum velocity during breaking - this is important for determining maximum thermal compensation
#define LEO_DXL_VOLTAGE_TEMP_FACT       ((1.0 + LEO_DXL_MAX_TEMP_DIFF*DXL_COPPER_COEF)/(1.0 + LEO_DXL_MAX_TEMP_DIFF*DXL_MAGNET_COEF))
#define LEO_DXL_VOLTAGE_TEMP_FACT_FULL  (LEO_DXL_VOLTAGE_TEMP_FACT + ((-1.0)*LEO_DXL_MAX_BREAKING_VEL*DXL_TORQUE_CONST*DXL_GEARBOX_RATIO/LEO_SUPPLY_VOLTAGE)*((1.0 + LEO_DXL_MAX_TEMP_DIFF*DXL_MAGNET_COEF) - LEO_DXL_VOLTAGE_TEMP_FACT))

// Define the maximum allowable Dynamixel voltage that can be guaranteed
// under all temperature compensation situations.
#define LEO_MAX_DXL_VOLTAGE             (LEO_SUPPLY_VOLTAGE/LEO_DXL_VOLTAGE_TEMP_FACT_FULL) // = 10.69 V @ RX-28;



// Dynamixel on, off and toggle convention
#define DXL_OFF                         0
#define DXL_ON                          1
#define DXL_TOGGLE                      2

#endif /* DYNAMIXELSPECS_H_ */
