/*
 * Simulator.cpp
 *
 *  Created on: Jan 11, 2010
 *      Author: wcaarls
 */

#include <grl/environments/odesim/simulator.h>

#define mLog ODESimulatorLog::mLog

using namespace grl;

ODESimulator::ODESimulator():
  CGenericODESim("simulator", 8),
  PosixNonRealTimeThread("simulator-thread", BaseThread::NORMAL),
  mEvtActuation(false, false),
  mRandomize(0),
  mActuationDelay(0)
{
  mLogDebugLn("Simulator constructor");

  mStepCounter        = 0;
  //resetBindData();
  resetActuationValues();
}

bool ODESimulator::readConfig(const CConfigSection &configSection, bool noObjects)
{
  bool result = true;
  result &= CSTGODESim<GenericState>::readConfig(configSection, noObjects);
  configSection.get("actuationdelay", &mActuationDelay);

  // Check that the actuation delay is not set larger than the step time
  if (mActuationDelay > mSim.getStepTime())
  {
    mLogErrorLn("Actuation delay (" << mActuationDelay << ") cannot be larger than step time (" << mSim.getStepTime() << ")!");
    return false;
  }

  return result;
}

// Waits and returns true when new actuation signals have arrived
bool ODESimulator::shouldStep()
{
  // Wait for the event to become signaled
  return mEvtActuation.wait();
}

bool ODESimulator::start()
{
  // Clear state and internal actuation values
  resetActuationValues();
  mState.clear();
  // Clear the actuation event
  mEvtActuation = false;
  // Clear state counter
  mStepCounter = 0;

  mSim.pause(true);
  bool result = mSim.start();
  PosixNonRealTimeThread::start();
  return result;
}

bool ODESimulator::stop()
{
  // Signal the thread that it should stop
  PosixNonRealTimeThread::stop();
  // Force the loop to continue
  mEvtActuation = true;
  // Await termination
  PosixNonRealTimeThread::awaitTermination();
  return mSim.stop();
}

void ODESimulator::fillState()
{
  // Fill generic state information
  CGenericODESim::fillState();

  // Set state ID
  mState.mStateID = mStepCounter++;
}

void ODESimulator::run()
{
  // Save original simulation timing
  int subsamplingFactor	= mSim.getSubsamplingFactor();
  double totalSteptime	= mSim.getStepTime();

  // Split original timing into two parts, while maintaining the same partial step time or better (totalSteptime/subsamplingFactor):
  // - actuation delay: ad
  // - remaining step: rs
  double adSteptime       = mActuationDelay;
  int adSubsamplingFactor	= (int)ceil(subsamplingFactor*mActuationDelay/totalSteptime);
  double rsSteptime       = totalSteptime - mActuationDelay;
  int rsSubsamplingFactor = (int)ceil(subsamplingFactor*rsSteptime/totalSteptime);
  mLogInfoLn("Timing (" << totalSteptime << "s in " << subsamplingFactor << "steps) is split into actuation delay (" << adSteptime << "s in " << adSubsamplingFactor << " steps) and a rest step (" << rsSteptime << "s in " << rsSubsamplingFactor << " steps)");

  // Broadcast the initial state of the robot so that the policy
  // can calculate the initial actuation signals.
  // shouldStep() will wait for the initial actuation signals from the policy (it is reset in start())
  setInitialCondition(mRandomize?time(NULL):0);

  mLogNoticeLn("Simulator settings:\n      step time: " << mSim.getStepTime() << ", subsamplingfactor: " << mSim.getSubsamplingFactor());

  fillState();

  // Simulate the change of a state in the case of a non-zero control delay
  if (adSubsamplingFactor > 0)
  {
    mSim.setTiming(adSteptime, adSubsamplingFactor);
    mSim.singleStep();
    mLogCrawlLn("Performed actuation delay step of " << adSteptime << " (subsampled " << adSubsamplingFactor << "x)");
  }

  if (!broadcast())
    mLogWarningLn("STG state broadcast failed on one of the receivers (error " << errno << ")!");

  // Main loop
  mLogDebugLn("Simulator::run() is entering main loop...");
  while (!shouldStop())
  {
    if (shouldStep())
    {
      // Make a simulation step
      if (rsSubsamplingFactor > 0)
      {
        mSim.setTiming(rsSteptime, rsSubsamplingFactor);
        mSim.singleStep();
      }

      // Since the step is done now, we don't need to ask for sim access, so just fill the state struct
      fillState();

      // Simulate the change of a state in case of control delay
      if (adSubsamplingFactor > 0)
      {
        mSim.setTiming(adSteptime, adSubsamplingFactor);
        mSim.singleStep();
      }

      // Now send our state to the queue
      if (broadcast())
      {
        //printf("STG state sent with ID %u!\n", mState.mStateID);
      }
      else
        mLogWarningLn("STG state broadcast failed on one of the receivers (error " << errno << ")!");
    }
  }
  mLogNoticeLn("Simulator::run() has come to an end.");
}

// Call activateActions() at a moment of your choosing to put the new control actions into effect
int ODESimulator::activateActions(const uint64_t& stateID)
{
  CGenericODESim::activateActions(stateID);

  // After we're done, signal (the main loop) that new actuation information is available
  mEvtActuation = true;
  return 0;	// 0 means success
}

void ODESimulator::setInitialCondition(long int seed)
{
  CGenericODESim::setInitialCondition(seed);

  // Acquire sim access so that we are not altering ODE objects while they are drawn, for example
  CODESimAccess simAccess(getSim());

  CODEObject* robot = simAccess.resolveObject("robot");
  if (robot == NULL)
  {
    mLogWarningLn("Robot is null!");
    return; // Something bad is going on
  }

  robot->setInitialCondition(seed);
}
