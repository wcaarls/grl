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
    CLog2				mLog;
    CODEHingeJoint*		mMotorJoints[ljNumDynamixels];
    CODEBody*			mBodyParts[lpNumParts];
    CODEGeom*			mFootContactGeoms[lfNumFootContacts];
    CSimVisObject*		mErrorLEDs[3];
//    CWaitEvent			mEvtActuation;
//		double				mActuationDelay;	// In microseconds
//		double				mPeriod;			// Backup of the step time / period
    const char*		getFootContactName(int footContactIndex);
    const char*		getBodyPartName(int partIndex);
    void			resetBindData();

    CLeoState mState;
    // Actuation values
    double				mMotorJointVoltages[ljNumDynamixels];

	public:
    CSTGLeoSim();
    ~CSTGLeoSim() {}

    void          setJointVoltage(int jointIndex, double voltage);					// Voltage in [V]
    double        getJointVoltage(int jointIndex);

		// It's wise to implement the physical limits of the motors, as well as their name
    double        getJointMaxVoltage(int jointIndex);
    bool			bindRobot(CODESim *sim);		// Bind elements (joints, bodies,..) from XML-read robot to the expected ones for Leo
    void          fillState(CLeoState &state);
//    void          copyState(CLeoState state) {mState = state;}
};

#endif /* STGLEOSIM_H_ */
