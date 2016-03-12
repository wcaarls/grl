/*
 * TLMModel.cpp
 *
 *  Created on: Sep 23, 2010
 *      Author: Erik Schuitema
 */

#include <grl/environments/compass_walker/SWModel.h>
#include <iomanip>
#include <iostream>
// **************************************************************** //
// ************************ CSWModelState ************************* //
// **************************************************************** //

void CSWModelState::init(double xOffset, double stanceLegAngle, double stanceLegAngleRate, double hipAngle, double hipAngleRate)
{
  mStanceLegAngle     = stanceLegAngle;
  mStanceLegAngleRate = stanceLegAngleRate;
  mHipAngle           = hipAngle;
  mHipAngleRate       = hipAngleRate;
  mStanceFootX        = xOffset;
  mStanceLegChanged   = false;
}

// **************************************************************** //
// *************************** CSWModel *************************** //
// **************************************************************** //

double CSWModel::detectEvents(const CSWModelState& stateT0, CSWModelState& stateT1, double hipTorque, double dt) const
{
  if ((stateT0.getSwingFootY() >= 0) && (stateT1.getSwingFootY() < 0))  // Swing foot passed through the floor
    if ( ((stateT0.mHipAngle < 0) && (stateT1.mHipAngle < 0))  ||
       ((stateT0.mHipAngle > 0) && (stateT1.mHipAngle > 0)) )  // Not parallel
       if ( ((stateT1.mStanceLegAngleRate < 0) && (stateT1.mHipAngle < 0)))// || // forward step in direction of movement
         // ((mState.mStanceLegAngleVel > 0) && (mState.mHipAngle > 0)) )  // backward step in the direction of movement
      {
        double timeleft = processStanceLegChange(stateT0, stateT1, hipTorque, dt);
        stateT1.mStanceLegChanged = true;
        return timeleft;
      }

  stateT1.mStanceLegChanged = false;
  return 0;
}

CSWModel::CSWModel():
  mStepTime(0.1*1E6),
  mNumIntegrationSteps(8),
  mHeelStrikeHeightPrecision(1.0E-11)  // 0.01 mm by default
{
}

CSWModel::~CSWModel()
{

}

double CSWModel::detectHeelstrikeMoment(const CSWModelState& stateT0, const CSWModelState& stateT1, CSWModelState& heelstrikeState, double hipTorque, double heightPrecision, double dt) const
{
  double timeLeft;  // Return the time left to integrate

  // The moment of heelstrike will be between s0 and s1. s0 is above floorlevel, s1 below.
  CSWModelState s0(stateT0), s1(stateT1);
  double  s0time(0), s1time(dt);
  const int maxIterations = 10;

  int iIter;
  for (iIter=0; iIter<maxIterations; iIter++)
  {
    heelstrikeState = s0;
    double newDt = (s1time - s0time) * s0.getSwingFootY() / (s0.getSwingFootY() - s1.getSwingFootY());
    integrateRK4(heelstrikeState, hipTorque, newDt);
    if (heelstrikeState.getSwingFootY() > 0)
    {
      s0 = heelstrikeState;
      s0time  = s0time + newDt;
    }
    else
    {
      s1 = heelstrikeState;
      s1time = s0time + newDt;
    }

    // Check if we are done yet
    if (s0.getSwingFootY() < heightPrecision)
    {
      heelstrikeState = s0;
      timeLeft    = dt - s0time;
      break;
    }
    else if (-s1.getSwingFootY() < heightPrecision)
    {
      heelstrikeState = s1;
      timeLeft    = dt - s1time;
      break;
    }
  }
  if (iIter >= maxIterations)
  {
    //mLogWarningLn("Couldn't reach desired heel strike precision (" << heightPrecision << ") within " << maxIterations << " iterations; reached " << fabs(heelstrikeState.getSwingFootY()) << "; best: " << std::min(fabs(s0.getSwingFootY()), fabs(s1.getSwingFootY())));
    if (heelstrikeState.getSwingFootY() > 0)
      timeLeft    = dt - s0time;
    else
      timeLeft    = dt - s1time;
  }
  //else
  //  mLogInfoLn("Determined heel strike in " << iIter << " iterations");
  return timeLeft;
}

double CSWModel::processStanceLegChange(const CSWModelState& stateT0, CSWModelState& stateT1, double hipTorque, double dt) const
{
  CSWModelState heelstrikeState;  // This state will receive the state at which heel strike took place
  double timeleft = detectHeelstrikeMoment(stateT0, stateT1, heelstrikeState, hipTorque, mHeelStrikeHeightPrecision, dt);

  // Fill the new state (stateT1) with the state after the impact
  // Adjust velocities
  stateT1.mHipAngleRate       = heelstrikeState.mStanceLegAngleRate*(cos(2.0*heelstrikeState.mStanceLegAngle)*(1.0 - cos(2.0*heelstrikeState.mStanceLegAngle)));
  stateT1.mStanceLegAngleRate = heelstrikeState.mStanceLegAngleRate*(cos(2.0*heelstrikeState.mStanceLegAngle));

  // Switch leg angles and set new stance foot position
  stateT1.mStanceFootX    =  heelstrikeState.getSwingFootX();
  stateT1.mStanceLegAngle = -heelstrikeState.mStanceLegAngle;
  stateT1.mHipAngle       = -2.0*heelstrikeState.mStanceLegAngle;  // Force hip angle to twice the stanceleg angle

  // save state right before heelstrike
  writeFile(dt - timeleft, heelstrikeState.mStanceLegAngle, heelstrikeState.mHipAngle, heelstrikeState.mStanceLegAngleRate,
            heelstrikeState.mHipAngleRate, 0, hipTorque);
  // save state right after heelstrike
  writeFile(dt - timeleft, stateT1.mStanceLegAngle, stateT1.mHipAngle, stateT1.mStanceLegAngleRate,
            stateT1.mHipAngleRate, 1, hipTorque);

  // We made a step!
  //mNumFootsteps++;
  return timeleft;
}

void CSWModel::setTiming(double steptime, int numIntegrationSteps)
{
  mStepTime = (uint64_t)floor((steptime + 0.5E-6)*1E6);
  mNumIntegrationSteps = numIntegrationSteps;
}

void CSWModel::setSlopeAngle(double angle)
{
  mSlopeAngle = angle;
}

double CSWModel::getSlopeAngle() const
{
  return mSlopeAngle;
}

std::ofstream stream_;

void CSWModel::openFile(const std::string fname) const
{
  stream_.open (fname, std::ofstream::out);
  stream_ << "COLUMNS:" << std::endl;
  stream_ << "time[0], " << std::endl;
  stream_ << "state[0], " << std::endl;
  stream_ << "state[1], " << std::endl;
  stream_ << "state[2], " << std::endl;
  stream_ << "state[3], " << std::endl;
  stream_ << "state[4], " << std::endl;
  stream_ << "action[0], " << std::endl;
  stream_ << "DATA:" << std::endl;
  time_ = 0;
}

void CSWModel::closeFile() const
{
  stream_.close();
}

bool CSWModel::writeFile(double timeOffset, double sla, double ha, double slar, double har, double contact, double torque) const
{

  if (stream_.is_open())
  {
    double time = time_ + timeOffset;
    stream_ << std::fixed << std::setw(11) << std::setprecision(6) << time     << ", "
            << std::fixed << std::setw(11) << std::setprecision(6) << sla      << ", "
            << std::fixed << std::setw(11) << std::setprecision(6) << ha       << ", "
            << std::fixed << std::setw(11) << std::setprecision(6) << slar     << ", "
            << std::fixed << std::setw(11) << std::setprecision(6) << har      << ", "
            << std::fixed << std::setw(11) << std::setprecision(6) << contact  << ", "
            << std::fixed << std::setw(11) << std::setprecision(6) << torque   << std::endl;
    return true;
  }
  return false;
}

void CSWModel::singleStep(CSWModelState& state, double hipTorque) const
{
  // Keep previous state
  CSWModelState prevState = state;

  bool stancelegChanged = false;

  // Integrate a number of times
  double partialStepTime = 1.0E-6*mStepTime/mNumIntegrationSteps;
  for (int i=0; i<mNumIntegrationSteps; i++)
  {
    // Save to file the beginning of every subintegration interval
    time_ += partialStepTime;
    writeFile(0, state.mStanceLegAngle, state.mHipAngle, state.mStanceLegAngleRate,
              state.mHipAngleRate, 0, hipTorque);

    // Runge-Kutta!
    integrateRK4(state, hipTorque, partialStepTime);

    // Wrap angles to the domain of -Pi to Pi (is this really necessary each substep?)
    state.wrapAngles();

    // Detect footstrikes and undertake actions to alter our state
    double timeleft = detectEvents(prevState, state, hipTorque, partialStepTime);
    stancelegChanged |= (timeleft > 0);

    if (timeleft > 0)
    {
      integrateRK4(state, hipTorque, timeleft);  // --> after this function, mState contains the new state
      // Wrap angles to the domain of -Pi to Pi
      state.wrapAngles();
    }
    // Backup prevState
    prevState = state;
  }
  // Copy hiptorque and stance leg change into the state
  state.mActionTorque    = hipTorque;
  state.mStanceLegChanged  = stancelegChanged;
  //stateT1.mDisturbanceTorque = mTempDisturbanceTorque;
}

void CSWModel::getAccelerations(const CSWModelState &state, const double hipTorque, CSWAccels* accels) const
{
  accels->mStanceLegAccel = sin(state.mStanceLegAngle - mSlopeAngle);
  accels->mHipAccel       = sin(state.mHipAngle) * (SQR(state.mStanceLegAngleRate) - cos(state.mStanceLegAngle - mSlopeAngle)) + accels->mStanceLegAccel;
  accels->mHipAccel       += hipTorque;
  //accels->mStanceLegAccel    += mTempDisturbanceTorque;
}

void CSWModel::integrateRK4(CSWModelState &state, double hipTorque, const double dt) const 
{
  CSWModelState s1, s2, s3, s4;
  CSWAccels  k1, k2, k3, k4;  // not the 'real' k's, but k/dt and actually accelerations, more or less?
  s1 = state;
  getAccelerations(s1, hipTorque, &k1);

  // Estimate velocities
  s2.mStanceLegAngleRate  = s1.mStanceLegAngleRate+ (dt/2)*k1.mStanceLegAccel;
  s2.mHipAngleRate        = s1.mHipAngleRate    + (dt/2)*k1.mHipAccel;
  // Estimate Angles
  s2.mStanceLegAngle      = s1.mStanceLegAngle  + (dt/2)*s1.mStanceLegAngleRate;
  s2.mHipAngle            = s1.mHipAngle        + (dt/2)*s1.mHipAngleRate;
  getAccelerations(s2, hipTorque, &k2);

  // Estimate velocities
  s3.mStanceLegAngleRate  = s1.mStanceLegAngleRate  + (dt/2)*k2.mStanceLegAccel;
  s3.mHipAngleRate        = s1.mHipAngleRate        + (dt/2)*k2.mHipAccel;
  // Estimate Angles
  s3.mStanceLegAngle      = s1.mStanceLegAngle  + (dt/2)*s2.mStanceLegAngleRate;
  s3.mHipAngle            = s1.mHipAngle        + (dt/2)*s2.mHipAngleRate;
  getAccelerations(s3, hipTorque, &k3);

  // Estimate velocities
  s4.mStanceLegAngleRate  = s1.mStanceLegAngleRate  + (dt)*k3.mStanceLegAccel;
  s4.mHipAngleRate        = s1.mHipAngleRate        + (dt)*k3.mHipAccel;
  // Estimate Angles
  s4.mStanceLegAngle      = s1.mStanceLegAngle  + (dt)*s3.mStanceLegAngleRate;
  s4.mHipAngle            = s1.mHipAngle        + (dt)*s3.mHipAngleRate;
  getAccelerations(s4, hipTorque, &k4);

  // Final answer: u_(i+1) = u_i + (1/6) * (k1 + 2*k2 + 2*k3 + k4). But k is actually k/dt.
  // Estimate velocities
  state.mStanceLegAngleRate = s1.mStanceLegAngleRate+ (dt/6)*(k1.mStanceLegAccel + 2*k2.mStanceLegAccel + 2*k3.mStanceLegAccel + k4.mStanceLegAccel);
  state.mHipAngleRate       = s1.mHipAngleRate    + (dt/6)*(k1.mHipAccel + 2*k2.mHipAccel + 2*k3.mHipAccel + k4.mHipAccel);
  // Estimate Angles
  state.mStanceLegAngle     = s1.mStanceLegAngle  + (dt/6)*(s1.mStanceLegAngleRate + 2*s2.mStanceLegAngleRate + 2*s3.mStanceLegAngleRate + s4.mStanceLegAngleRate);
  state.mHipAngle           = s1.mHipAngle        + (dt/6)*(s1.mHipAngleRate + 2*s2.mHipAngleRate + 2*s3.mHipAngleRate + s4.mHipAngleRate);
}

