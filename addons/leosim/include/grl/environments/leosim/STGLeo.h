/*
 * STGleo.h
 *
 *  Created on: Nov 25, 2008
 *      Author: Erik Schuitema
 */

#ifndef STGLEO_H_
#define STGLEO_H_

#include <STG.h>
#include <sstream>
#include <string>
#include <bithacks.h>
#include "DynamixelSpecs.h"

#define LEO_NUM_FOOTSENSORS             4    // must be <= 8; there are 8 ADC channels
#define LEO_FOOTSENSOR_RIGHT_TOE        0x01
#define LEO_FOOTSENSOR_RIGHT_HEEL       0x02
#define LEO_FOOTSENSOR_LEFT_TOE         0x04
#define LEO_FOOTSENSOR_LEFT_HEEL        0x08
static const char* LEO_FOOTSENSOR_NAME[] = {"RightToe", "RightHeel", "LeftToe", "LeftHeel"};

#define LEO_SUPPLY_VOLTAGE              14.0    // Volts

// Temperature compensation defines
#define LEO_DXL_REF_TEMP                25.0    // Use 25.0 with copperfact=0.004 and magnetfact=0.0
#define LEO_DXL_MAX_TEMP                75.0
#define LEO_DXL_MAX_TEMP_DIFF           (LEO_DXL_MAX_TEMP - LEO_DXL_REF_TEMP)
#define LEO_DXL_MAX_BREAKING_VEL        4.0        // Maximum velocity during breaking - this is important for determining maximum thermal compensation
#define LEO_DXL_VOLTAGE_TEMP_FACT       ((1.0 + LEO_DXL_MAX_TEMP_DIFF*DXL_RX28_COPPER_COEF)/(1.0 + LEO_DXL_MAX_TEMP_DIFF*DXL_RX28_MAGNET_COEF))
#define LEO_DXL_VOLTAGE_TEMP_FACT_FULL  (LEO_DXL_VOLTAGE_TEMP_FACT + ((-1.0)*LEO_DXL_MAX_BREAKING_VEL*DXL_RX28_TORQUE_CONST*DXL_RX28_GEARBOX_RATIO/LEO_SUPPLY_VOLTAGE)*((1.0 + LEO_DXL_MAX_TEMP_DIFF*DXL_RX28_MAGNET_COEF) - LEO_DXL_VOLTAGE_TEMP_FACT))

// Define the maximum allowable Dynamixel voltage that can be guaranteed
// under all temperature compensation situations.
#define LEO_MAX_DXL_VOLTAGE             (LEO_SUPPLY_VOLTAGE/LEO_DXL_VOLTAGE_TEMP_FACT_FULL)

enum ELeoFootContact
{
    lfToeRight,
    lfHeelRight,
    lfToeLeft,
    lfHeelLeft,
    lfNumFootContacts,
    lfInvalid
};
//#define LEO_STG_QUEUE_NAME            "Leo-STG-Queue"

// Leo's joints enumeration
// Changing the order of these items *may* have severe consequences, e.g., when ljHipLeft is no longer zero.
enum ELeoJoint
{
    ljHipLeft,
    ljHipRight,
    ljKneeLeft,
    ljKneeRight,
    ljAnkleLeft,
    ljAnkleRight,
    ljNumLegJoints,
    ljShoulder = ljNumLegJoints,
    ljNumDynamixels,
    ljTorso = ljNumDynamixels,
    ljNumJoints,
    ljInvalid
};

enum ELeoPart
{
    lpTorso,
    lpUpperLegLeft,
    lpUpperLegRight,
    lpLowerLegLeft,
    lpLowerLegRight,
    lpFootLeft,
    lpFootRight,
    lpArm,
    lpNumParts,
    lpInvalid
};


class ISTGLeoActuation: public ISTGActuation
{
    public:
        const std::string    getJointName(int jointIndex)
        {
            switch (jointIndex)
            {
                case ljHipLeft:        return "hipleft"; break;
                case ljHipRight:    return "hipright"; break;
                case ljKneeLeft:    return "kneeleft"; break;
                case ljKneeRight:    return "kneeright"; break;
                case ljAnkleLeft:    return "ankleleft"; break;
                case ljAnkleRight:    return "ankleright"; break;
                case ljShoulder:    return "shoulder"; break;
                case ljTorso:        return "torso"; break;
                default:
                    return "";
            }
        }

        int                    getJointIndexByName(const std::string& jointName)
        {
            for (int i=0; i<ljNumJoints; i++)
                if (jointName.compare(getJointName(i)) == 0)
                    return (ELeoJoint)i;

            return ljInvalid;
        }
};

class CLeoState: public CSTGState
{
    public:
        double            mJointAngles[ljNumJoints];
        double            mJointSpeedsRaw[ljNumJoints];
        double            mJointSpeeds[ljNumJoints];
        // The actuation variables represent the actions taken BEFORE this state was received by any controller,
        // i.e., they don't represent the actions taken upon the state information in this state.
        double            mActuationAngles[ljNumDynamixels];
        double            mActuationSpeeds[ljNumDynamixels];
        double            mActuationVoltages[ljNumDynamixels];
        double            mActuationVoltagesTempComp[ljNumDynamixels];    // Temperature compensated
        uint64_t        mActuationDelay;
        unsigned char    mFootContacts;
        double            mFootSensors[LEO_NUM_FOOTSENSORS];

        CLeoState()
        {
            clear();    // Only calls CSTGState::clear() because this is the constructor?
        }

        void    clear()
        {
            CSTGState::clear();
            memset(mJointAngles, 0, ljNumJoints*sizeof(double));
            memset(mJointSpeedsRaw, 0, ljNumJoints*sizeof(double));
            memset(mJointSpeeds, 0, ljNumJoints*sizeof(double));
            memset(mActuationAngles, 0, ljNumDynamixels*sizeof(double));
            memset(mActuationSpeeds, 0, ljNumDynamixels*sizeof(double));
            memset(mActuationVoltages, 0, ljNumDynamixels*sizeof(double));
            memset(mActuationVoltagesTempComp, 0, ljNumDynamixels*sizeof(double));
            mFootContacts    = 0;
            mActuationDelay    = 0;
            memset(mFootSensors, 0, LEO_NUM_FOOTSENSORS*sizeof(double));
            //logCrawlLn(CLog2("stg"), "Cleared CLeoState");
            // Don't do this, because it bugs: memset(this, 0, sizeof(CLeoState));
        }

        // Headerline for printing the state data to a string
        const std::string toTextHeader()
        {
            ISTGLeoActuation leoAct;    // Dummy actuation interface
            std::stringstream out;
            // Index
            out << "ID\t";
            // Angles, raw speeds and filtered speeds + actuation signals
            for (int iJoint=0; iJoint<ljNumJoints; iJoint++)
            {
                std::string jointName = leoAct.getJointName(iJoint);
                out << jointName << "Angle" << "\t" << jointName << "SpeedRaw" << "\t" << jointName << "Speed" << "\t";
                if (iJoint < ljNumDynamixels)
                    out << jointName << "ActuationAngle" << "\t" << jointName << "ActuationSpeed" << "\t" << jointName << "ActuationVoltage" << "\t" << jointName << "ActuationVoltageTempComp" << "\t";
            }
            // Foot info
            for (int iFoot=0; iFoot<LEO_NUM_FOOTSENSORS; iFoot++)
            //    out << "FootSensor" << iFoot << "\t" << "FootContact" << iFoot << "\t";
                out << LEO_FOOTSENSOR_NAME[iFoot] << "Sensor\t" << LEO_FOOTSENSOR_NAME[iFoot] << "Contact\t";

            // Temperature
            //out << "Temp1\t" << "Temp2\t";
            out << std::endl;
            return out.str();
        }

        // Prints the state data to a string
        std::string toText()
        {
            std::stringstream out;
            // Index
            out << mStateID << "\t";
            // Angles, raw speeds and filtered speeds + actuation signals
            for (int iJoint=0; iJoint<ljNumJoints; iJoint++)
            {
                out << mJointAngles[iJoint] << "\t" << mJointSpeedsRaw[iJoint] << "\t" << mJointSpeeds[iJoint] << "\t";
                if (iJoint < ljNumDynamixels)
                    out << mActuationAngles[iJoint] << "\t" << mActuationSpeeds[iJoint] << "\t" << mActuationVoltages[iJoint] << "\t" << mActuationVoltagesTempComp[iJoint] << "\t";
            }
            // Foot info
            for (int iFoot=0; iFoot<LEO_NUM_FOOTSENSORS; iFoot++)
                out << mFootSensors[iFoot] << "\t" << B_IS_SET(mFootContacts, iFoot) << "\t";

            // Temperature
            //out << mTemp1 << "\t" << mTemp2 << "\t";
            // End
            out << std::endl;
            return out.str();
        }

        // Joint index transform functions (static)
        static int mirrorJointIndex(const int jointIndex)
        {
            switch (jointIndex)
            {
                case ljHipLeft:
                    return ljHipRight;
                case ljHipRight:
                    return ljHipLeft;
                case ljKneeLeft:
                    return ljKneeRight;
                case ljKneeRight:
                    return ljKneeLeft;
                case ljAnkleLeft:
                    return ljAnkleRight;
                case ljAnkleRight:
                    return ljAnkleLeft;
                default:
                    return jointIndex;
            }
        }
};

typedef CStateTransitionGenerator<CLeoState> CSTGLeo;
typedef CSTGInQueue<CLeoState> CSTGLeoInQueue;

#endif /* STGLEO_H_ */
