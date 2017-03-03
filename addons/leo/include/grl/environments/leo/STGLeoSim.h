/*
 * STGLeoSim.h
 *
 *  Created on: Apr 8, 2009
 *      Author: Erik Schuitema
 */

#ifndef STGLEOSIM_H_
#define STGLEOSIM_H_

#include <STGLeo.h>
#include <STGODESim.h>

class CSTGLeoSim: public ISTGLeoActuation
{
	protected:
    double				mMotorJointVoltages[ljNumDynamixels];
    double				mMotorJointTorques[ljNumDynamixels];

	public:
    CSTGLeoSim() {}
    void          setJointVoltage(int jointIndex, double voltage);				// in [V]
    double        getJointVoltage(int jointIndex);
    double        getJointMaxVoltage(int jointIndex);

    void          setJointTorque(int jointIndex, double torque);					// in [Nm]
    double        getJointTorque(int jointIndex);
//    double        getJointMaxVoltage(int jointIndex);
};

#endif /* STGLEOSIM_H_ */
