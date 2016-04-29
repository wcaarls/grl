/*
 * SWModel.h
 *
 * Analytical Agent Environment Model (state transition model) for the Simplest Walker
 *
 *  Created on: Apr 1, 2011
 *      Author: Erik Schuitema
 */

#ifndef SWMODEL_H_
#define SWMODEL_H_

#include <math.h>
#include <stdint.h>
#include <fstream>
#include <grl/environment.h>

#ifndef SQR
  #define SQR(X) ((X)*(X))
#endif

class CSWModelState
{
  public:
    double    mStanceLegAngle;
    double    mStanceLegAngleRate;
    double    mHipAngle;
    double    mHipAngleRate;
    double    mStanceFootX;

    double    mActionTorque;      // On the swing leg
    double    mDisturbanceTorque; // On the stance leg

    bool      mStanceLegChanged;

    void      init(double xOffset, double stanceLegAngle, double stanceLegAngleRate, double hipAngle, double hipAngleRate);

    // The stance leg angle is absolute (= angle with the normal to the floor)
    // The absolute angle of the swing leg is: mStanceLegAngle - mHipAngle
    inline double  getHipX() const                  {return mStanceFootX - sin(mStanceLegAngle);}
    inline double  getHipY() const                  {return cos(mStanceLegAngle);}
    inline double  getSwingFootX() const            {return getHipX() + sin(mStanceLegAngle - mHipAngle);}
    inline double  getSwingFootY() const            {return getHipY() - cos(mStanceLegAngle - mHipAngle);}
    inline virtual double  getKinEnergy() const     {return SQR(mStanceLegAngleRate)/2.0;}
    inline double  getPotEnergy(double gamma) const {return getHipY()*cos(gamma) - getHipX()*sin(gamma);}  // gamma indicates slope angle
    inline double  getEnergy(double gamma) const    {return getKinEnergy() + getPotEnergy(gamma);}

    void wrapAngles()
    {
      if (mStanceLegAngle >= M_PI)
        mStanceLegAngle -= 2*M_PI;
      if (mStanceLegAngle < -M_PI)
        mStanceLegAngle += 2*M_PI;

      if (mHipAngle >= M_PI)
        mHipAngle -= 2*M_PI;
      if (mHipAngle < -M_PI)
        mHipAngle += 2*M_PI;
    }
};

class CSWAccels
{
  public:
    double  mStanceLegAccel;
    double  mHipAccel;
};

// State dim index enumeration for agent state
enum ESWStateDimIndex
{
  siStanceLegAngle,
  siHipAngle,
  siStanceLegAngleRate,
  siHipAngleRate,
  siNumStateDims
};

class CSWModel
{
  protected:
    // Simulation params
    double      mSlopeAngle;  // Floor
    uint64_t    mStepTime;    // In microseconds
    int         mNumIntegrationSteps;
    double      mHeelStrikeHeightPrecision;

    inline void getAccelerations(const CSWModelState &state, const double hipTorque, CSWAccels* accels) const;
    // integrateRK4() works with seconds (double precision) to increase precision of the detection of heelstrike
    void        integrateRK4(CSWModelState &state, double hipTorque, const double dt) const;
    // The three functions below return the integration time left after a possible heel strike event
    // dt is the time difference between stateT0 and stateT1.
    double      detectEvents(const CSWModelState& stateT0, CSWModelState& stateHS, CSWModelState& stateT1, double hipTorque, double dt) const;
    double      processStanceLegChange(const CSWModelState& stateT0, CSWModelState& stateHS, CSWModelState& stateT1, double hipTorque, double dt) const;
    double      detectHeelstrikeMoment(const CSWModelState& stateT0, const CSWModelState& stateT1, CSWModelState& heelstrikeState, double hipTorque, double heightPrecision, double dt) const;

  public:
    CSWModel();

    int         getNumIntegrationSteps() const  {return mNumIntegrationSteps;}
    uint64_t    getStepTimeUs() const           {return mStepTime;}  // Time in microseconds
    void        setTiming(double steptime, int numIntegrationSteps);
    void        setSlopeAngle(double angle);
    double      getSlopeAngle() const;

    void        singleStep(CSWModelState& state, double hipTorque, grl::Exporter * const exporter = NULL, double time=0) const;
};

#endif /* SWMODEL_H_ */
