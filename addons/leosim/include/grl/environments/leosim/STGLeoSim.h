/*
 * STGLeoSim.h
 *
 *  Created on: Apr 8, 2009
 *      Author: Erik Schuitema
 */

#ifndef STGLEOSIM_H_
#define STGLEOSIM_H_

#include "STGLeo.h"

class CSTGLeoSim: public ISTGLeoActuation
{
	protected:
		// Actuation values
		double				mMotorJointVoltages[ljNumDynamixels];

	public:
    CSTGLeoSim()  {}
    ~CSTGLeoSim() {}

    void          setJointVoltage(int jointIndex, double voltage);					// Voltage in [V]
    double        getJointVoltage(int jointIndex);

		// It's wise to implement the physical limits of the motors, as well as their name
    double        getJointMaxVoltage(int jointIndex);
};

#endif /* STGLEOSIM_H_ */
