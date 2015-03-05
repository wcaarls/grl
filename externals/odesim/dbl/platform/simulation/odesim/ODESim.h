/*
 *    Basic ODE simulator class designed for simulations of robots
 *    Copyright (C) 2012 Erik Schuitema (DBL)
 *
 *    This program is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU Lesser General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public License
 *    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef ODESIM_H_INCLUDED
#define ODESIM_H_INCLUDED

#include <ode/ode.h>
#include "ODEObjects.h"
#include <vector>
#include <PosixNonRealTimeThread.hpp>
#include <WaitEvent.hpp>
#include <Configuration.h>
#include "ODECollisions.h"
#include "ODELogging.h"
#include <SimVis.h>

struct SSimTimer
{
	bool	enabled;
	double	time;
};

#define CONST_ODESIM_DEFAULT_TIMER 0

class CODEConfig
{
	protected:

	public:
};

// Forward declaration
class CODESim;

class CODESimWorkerThread: public PosixNonRealTimeThread, public CODELoggable
{
	protected:
		CODESim		*mpSim;
		void		run();

	public:
		CODESimWorkerThread(CODESim* sim);

};

#define CONST_ODESIM_SINGLE_STEP_TIMEOUT	100000	// [milliseconds]

class CODESim: public CVisualSim, public CODELoggable
{
	friend class CODESimWorkerThread;
	friend class CODESimAccess;

	private:
		// World, main space. There is only one world
		dWorld						mWorld;

		//std::vector<dSpaceID>		mSpaces;		// There's a list of spaces

		// Timers
		std::vector<SSimTimer>		mTimers;

		// Settings
		double						mTotalStepTime;		// The total steptime
		int							mStepDelay;		// Not used yet
		bool						mRealtime;		// Not used yet
		int							mSubsamplingFactor;
		// Thread safe 'booleans' that can be waited on to be signaled
		CWaitEvent					mEvtRunContinuously;
		CWaitEvent					mEvtShouldStep;
		CWaitEvent					mEvtShouldStop;
		CWaitEvent					mEvtSingleStepDone;
		pthread_mutex_t				mStartStopMutex;	// Mutex to make starting and stopping the sim thread-safe

		// The worker thread
		CODESimWorkerThread			*mWorkerThread;
		pthread_mutex_t				mSimAccessMutex;
		bool						mSimIsBeingAccessed;
		bool                        mODEError;

	protected:
		// Body that represents the world (id=0). Used for joints; contains anchors for joints with the fixed world
		// Don't call mWorldBody.create() nor init()!!
		CODEBody					mWorldBody;
		std::vector<CODEObject*>	mObjects;
		double						mGravityX, mGravityY, mGravityZ;
		double						mGlobalERP;
		double						mGlobalCFM;
		CODECollisionHandler		mCollisionHandler;
		bool						mCollisionsInvalid;	// Indicates whether collisions should be calculated *before* the next simulation step (default is after)

		// Loop control
		// Partial step time: the simulator will eventually call dWorldStep(getPartialStepTime());
		inline double				getPartialStepTime()	{return mTotalStepTime/mSubsamplingFactor;}
		bool						shouldContinue();	// Waits if necessary and returns true if worker thread should continue
		void						incrementTime();
		void						updateJointMotors(double stepTime);
		void						updateExternalForces();
		void						step();
		void						calculateCollisions(CODESimAccess *simAccess);
		CODEObject*					resolveObject(const std::string &objectName);	// Not public; see CODESimAccess

	public:
		CODESim();
		virtual ~CODESim();

		// Init/deinit
		virtual bool	readConfig(const CConfigSection &configSection, bool noObjects=false);
		void			convertKDToERPCFM(double K, double D, double *ERP, double *CFM);
		//void			getERPCFM(double *ERP, double *CFM);
		void			setGlobalParamsKD(double globalK, double globalD);
		void			setGlobalParamsERPCFM(double globalERP, double globalCFM);
		void			setGravity(double gx, double gy, double gz);

		dWorldID		getWorldID()	{return mWorld.id();}	// ID of the world
		CODEBody*		getWorldBody()	{return &mWorldBody;}	// CODEBody that represents the world (its ID is 0)

		bool			init();
		bool			deinit();

		void			clearAll();	// Reset the sim to a completely empty initial state


		// Simulation step time.
		// If you ever want to implement a setStepTime(),
		// make sure you recalculate mGlobalERP and mGlobalCFM
		// if the user provided globalK and globalD parameters!
		double			getStepTime();

		// !!!WARNING!!!
		// If you change the timing while simulating, make sure that totalStepTime/subsamplingFactor
		// remains equal to its value before the change, so that all convertKDToERPCFM() calculations
		// that were performed in readConfig() functions all over the place will still be valid!!!!
		// !!!WARNING!!!
		void			setTiming(double totalStepTime, int subsamplingFactor);

		void			setSubsamplingFactor(const int subsamplingFactor);
		int				getSubsamplingFactor();


		void			setStepDelay(int numMilliSeconds);
		int				getStepDelay();
		//void			singleStep();		// Takes a single simulation step. This function is guaranteed to return after the step is actually performed. Called OUTSIDE any ODE
		//void			pause(bool enablePause=true);
		bool			isRealtime();
		bool			setRealtime(bool bEnabled);

		// The timers! Timers are convenient to keep track of the length of (sub)processes
		// within your simulation, for example a robot's foot step time or the time needed to reach a goal.
		// Timers cannot be deleted (to be able to preserve the timer indices).
		// Timer CONST_ODESIM_DEFAULT_TIMER is always there.
		// Use addTimer() to request a new timer index
		int				addTimer();

		double			getTime(const int timerIndex=0);
		void			enableTimer(const int timerIndex=0);
		void			disableTimer(const int timerIndex=0);
		void			resetTime(const int timerIndex=0);
		void			clearTimers();

		// Function to add new objects (like robots)
		// NOTE: CODESim becomes OWNER of the added objects and will destroy them in deinit()
		void			addObject(CODEObject *pObject);
		void			clearObjects();

		bool			start();		// When the sim is started, calculateCollisions() is called before making the first step (and also before preStep()).
		bool			stop();
		void			pause(bool shouldPause);
		bool			singleStep();	// Take a single simulation step. Returns true on success; returns false on timeout or when not-ready for a new step


		// If any geom is moved or rotated (either directly, or through the attached body),
		// this function should be called so that collision contacts are properly recalculated
		// before the next simulation step.
		void			invalidateCollisions(bool invalid=true);

		// Override these functions to execute code before or after collision detection
		virtual void	onPreCollide()	{}
		virtual void	onPostCollide()	{}


		// Override these functions to execute code before or after the integration step
		virtual void	onPreStep()		{}	// This is a good place to insert actuation code and code that simulates springs and locks
		virtual void	onPreSubStep()	{}	// This function is called before every SUB-step of the simulation
		virtual void	onPostSubStep()	{}	// This function is called after every SUB-step of the simulation
		virtual void	onPostStep()	{}	// This is a good place to signal a GUI or control loop that new data is available

		// Visualization
		void			getSimVisObjects(CSimVisObjectPtrArray* objects);
};

class CODESimAccess: public CGenericSimAccess
{
	friend class CODESim;

	public:
	/*
		CODESimAccess():
			CVisualSimAccess()		{}
			*/
		CODESimAccess(CODESim *sim):
			CGenericSimAccess(sim)	{}

		const std::vector<CODEObject*>& getObjects();
		CODEObject*						resolveObject(const std::string &objectName);
		CODECollisionHandler*			getCollisionHandler();
};

#endif /* ODESIM_H_ */
