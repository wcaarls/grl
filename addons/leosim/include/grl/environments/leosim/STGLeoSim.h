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
//#include <ThirdOrderButterworth.h>
//#include <grl/environments/odesim/simulator.h>

class CSTGLeoSim: public ISTGLeoActuation //public CSTGODESim<CLeoState>, public ISTGLeoActuation, private PosixNonRealTimeThread
{
	protected:
//    CLog2				mLog;

    double				mMotorJointVoltages[ljNumDynamixels];

	public:
    CSTGLeoSim() {}
    ~CSTGLeoSim() {}

    void          setJointVoltage(int jointIndex, double voltage);					// Voltage in [V]
    double        getJointVoltage(int jointIndex);
    double        getJointMaxVoltage(int jointIndex);

};

#endif /* STGLEOSIM_H_ */
