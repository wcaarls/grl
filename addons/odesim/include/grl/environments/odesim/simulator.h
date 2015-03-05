#ifndef ODESIM_SIMULATOR_H_
#define ODESIM_SIMULATOR_H_

#include <GenericODESim.h>

namespace grl
{

class ODESimulatorLog
{
  protected:
    CLog2 mLog;
  public:
    ODESimulatorLog() : mLog("sim") { }
};

class ODESimulator: public CGenericODESim, private PosixNonRealTimeThread, protected ODESimulatorLog
{
  protected:
    uint64_t    mStepCounter;
    CWaitEvent  mEvtActuation;

    //void      resetBindData();
    // Main function from AbstractXenomaiTask
    // that has to be overridden in order to
    // implement actual task functionality.
    void      run();
    bool      shouldStep(); // Waits for an actuation signal and returns true when a new step should be made
    void      fillState();

  public:
    ODESimulator();

    bool      start();
    bool      stop();
    bool      isRunning()
    {
            return running();
    }

    virtual void setInitialCondition(long int seed=0);

    virtual bool readConfig(const CConfigSection &configSection, bool noObjects=false);

    // Call activateActions() at a moment of your choosing to put the new control actions into effect
    virtual int activateActions(const uint64_t& stateID);
};

}

#endif /* ODESIM_SIMULATOR_H_ */
