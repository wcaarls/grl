/*
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

#include "ODESim.h"

//****************************** CODESimWorkerThread ******************************//


CODESimWorkerThread::CODESimWorkerThread(CODESim* sim):
	PosixNonRealTimeThread("ODEWorkerThread", BaseThread::NORMAL)
{
	mpSim = sim;
}

void CODESimWorkerThread::run()
{
	mLogDebugLn("CODESimWorkerThread::run() started.");
	while (mpSim->shouldContinue())
	{
		//printf("CODESimWorkerThread::run() starting sim step.\n");
		mpSim->step();
		//printf("CODESimWorkerThread::run() finished sim step.\n");
	}
	mLogDebugLn("CODESimWorkerThread::run() ended.");
}

//*************************** CODESimAccess ***************************//

const std::vector<CODEObject*>& CODESimAccess::getObjects()
{
	return ((CODESim*)mpSim)->mObjects;
}

CODEObject* CODESimAccess::resolveObject(const std::string &objectName)
{
	return ((CODESim*)mpSim)->resolveObject(objectName);
}

CODECollisionHandler* CODESimAccess::getCollisionHandler()
{
	return &((CODESim*)mpSim)->mCollisionHandler;
}

//****************************** CODESim ******************************//

CODESim::CODESim():
	mEvtRunContinuously(true, false),
	mEvtShouldStep(false, false),
	mEvtShouldStop(true, false),
	mEvtSingleStepDone(true, true),	// manual reset, set to true initially
    mODEError(false),
	mWorldBody(NULL),
	mCollisionHandler(this),
	mCollisionsInvalid(true)
{
	// ODE variables
	mGravityX			= mGravityY = mGravityZ = 0;
	mGlobalERP			= 0.2;
	mGlobalCFM			= 0.0;
	mTotalStepTime		= 0.1;
	mStepDelay			= 0;
	mRealtime			= false;
	mSubsamplingFactor	= 1;

	mWorldBody.setName("world");

	// Worker thread, and mutex to start/stop it
	mWorkerThread		= NULL;
	pthread_mutex_init(&mStartStopMutex, NULL);
	pthread_mutex_init(&mSimAccessMutex, NULL);
	mSimIsBeingAccessed	= false;

	// Reset all timers and initialize the default one
	clearTimers();
}

CODESim::~CODESim()
{
	stop();	// Stop sim; doesn't harm if sim already stopped
	clearObjects();
	pthread_mutex_destroy(&mStartStopMutex);
	pthread_mutex_destroy(&mSimAccessMutex);
}

void CODESim::clearObjects()
{
	for (unsigned int iObject=0; iObject<mObjects.size(); iObject++)
	{
		mObjects[iObject]->deinit();
		delete mObjects[iObject];
	}
	mObjects.clear();
}

CODEObject* CODESim::resolveObject(const std::string &objectName)
{
	CODEObject* foundObject = NULL;

	for (unsigned int iObject=0; iObject<mObjects.size(); iObject++)
	{
		if (mObjects[iObject]->name() == objectName)
		{
			foundObject = mObjects[iObject];
			break;
		}
	}
	return foundObject;
}

double CODESim::getStepTime()
{
	return mTotalStepTime;
}

void CODESim::setTiming(double totalStepTime, int subsamplingFactor)
{
	mTotalStepTime		= totalStepTime;
	mSubsamplingFactor	= subsamplingFactor;

	mLogDebugLn("Simulator step time set to " << getPartialStepTime() << "*" << getSubsamplingFactor() << " = " << getStepTime());
}

void CODESim::setSubsamplingFactor(const int subsamplingFactor)
{
	mSubsamplingFactor = subsamplingFactor;

	mLogDebugLn("Simulator step time set to " << getPartialStepTime() << "*" << getSubsamplingFactor() << " = " << getStepTime());
}

int CODESim::getSubsamplingFactor()
{
	return mSubsamplingFactor;
}

void CODESim::setStepDelay(int numMilliSeconds)
{
	mStepDelay = numMilliSeconds;
}

int CODESim::getStepDelay()
{
	return mStepDelay;
}

bool CODESim::isRealtime()
{
	return mRealtime;
}

bool CODESim::setRealtime(bool enabled)
{
	return mRealtime = enabled;
}

int CODESim::addTimer()
{
	SSimTimer newTimer;
	newTimer.time = 0;
	newTimer.enabled = false;
	mTimers.push_back(newTimer);
	return (int)mTimers.size() - 1;
}

double CODESim::getTime(const int timerIndex)
{
	return mTimers[timerIndex].time;
}

void CODESim::resetTime(const int timerIndex)
{
	mTimers[timerIndex].time = 0;
}

void CODESim::enableTimer(const int timerIndex)
{
	mTimers[timerIndex].enabled = true;
}

void CODESim::disableTimer(const int timerIndex)
{
	mTimers[timerIndex].enabled = false;
}

void CODESim::clearTimers()
{
	mTimers.clear();
	// Add the default timer
	addTimer();
	// Reset and enable it
	resetTime(CONST_ODESIM_DEFAULT_TIMER);
	enableTimer(CONST_ODESIM_DEFAULT_TIMER);
}

void CODESim::incrementTime()
{
	for (unsigned int i=0; i<mTimers.size(); i++)
		if (mTimers[i].enabled)
			mTimers[i].time += mTotalStepTime;
}

void CODESim::addObject(CODEObject *pObject)
{
	mObjects.push_back(pObject);
}

bool CODESim::readConfig(const CConfigSection &configSection, bool noObjects)
{
	bool configresult = true;

	// Acquire sim access (i.e., don't change the configuration in the middle of a simulation step
	CODESimAccess simAccess(this);

	// Clear everything before reading a new configuration
	clearAll();

	configresult &= mLogAssert(configSection.get("steptime", &mTotalStepTime));
	configresult &= mLogAssert(configSection.get("subsamplingfactor",  &mSubsamplingFactor));

	// Missing gravity information should not turn configResult into false
	configSection.get("gravityX",  &mGravityX);
	configSection.get("gravityY",  &mGravityY);
	configSection.get("gravityZ",  &mGravityZ);

	// Make sure to set the step time and subsampling factor BEFORE reading globalK and globalD!!
	if (configSection.has("globalK"))
	{
		double globalK, globalD;
		configresult &= mLogAssert(configSection.get("globalK",  &globalK));
		configresult &= mLogAssert(configSection.get("globalD",  &globalD));
		setGlobalParamsKD(globalK, globalD);
	}
	else
	if (configSection.has("globalERP"))
	{
		configresult &= mLogAssert(configSection.get("globalERP",  &mGlobalERP));
		configresult &= mLogAssert(configSection.get("globalCFM",  &mGlobalCFM));
	}
	mLogNoticeLn("Global sim params: ERP:" << mGlobalERP << ", CFM:" << mGlobalCFM);

	// Read anchors and drawing objects for the mWorldBody object by feeding this noe to mWorldBody.
	// We should ignore the readConfig() result because not everything can be read for the mWorld body.
	mWorldBody.readAnchorsConfig(configSection);
	mWorldBody.readDrawingObjectsConfig(configSection);

	if (!noObjects)
	{
		// Read objects as well
		for (CConfigSection objectNode = configSection.section("object"); !objectNode.isNull(); objectNode = objectNode.nextSimilarSection())
		{
			CODEObject *newObject = new CODEObject(this);
			if (!newObject->readConfig(objectNode))
			{
				mLogErrorLn("CODESim::readConfig() failed to read object " << newObject->name() << "!");
				configresult = false;
			}
			addObject(newObject);
		}
	}

	// Read collision info. The collision handler will (should!) clear itself before reading a new configuration.
	mCollisionHandler.readConfig(configSection.section("collisions"), &simAccess);

	// Release sim access is done in the destructor of CODESimAccess
	return configresult;
}

void CODESim::convertKDToERPCFM(double K, double D, double *ERP, double *CFM)
{
	double partialStepTime = getPartialStepTime();
	*ERP = partialStepTime*K/(partialStepTime*K + D);
	*CFM = 1.0/(partialStepTime*K + D);
}

//void CODESim::getERPCFM(double *ERP, double *CFM)
//{
//	*ERP = mGlobalERP;
//	*CFM = mGlobalCFM;
//}

void CODESim::setGlobalParamsKD(double globalK, double globalD)
{
	convertKDToERPCFM(globalK, globalD, &mGlobalERP, &mGlobalCFM);
}

void CODESim::setGlobalParamsERPCFM(double globalERP, double globalCFM)
{
	mGlobalERP = globalERP;
	mGlobalCFM = globalCFM;
}

void CODESim::setGravity(double gx, double gy, double gz)
{
	mGravityX = gx;
	mGravityY = gy;
	mGravityZ = gz;
}

bool CODESim::init()
{
	dInitODE2(0);
	mWorld.setGravity(mGravityX, mGravityY, mGravityZ);
	mWorld.setERP(mGlobalERP);
	mWorld.setCFM(mGlobalCFM);

	for (unsigned int iObject=0; iObject<mObjects.size(); iObject++)
		mObjects[iObject]->init(mWorld);

	setInitialized(true);
	return true;
}

bool CODESim::deinit()
{
	if (isInitialized())
	{
		CODESimAccess simAccess(this);
		setInitialized(false);

		clearAll();
		mCollisionHandler.clearAll();
		dCloseODE();
	}

	return true;
}

// Reset the sim to a completely empty initial state
void CODESim::clearAll()
{
	clearTimers();
	clearObjects();
	mGravityX = mGravityY = mGravityZ = 0;
}

void CODESim::updateJointMotors(double stepTime)
{
	for (unsigned int iObject=0; iObject<mObjects.size(); iObject++)
		for (unsigned int iJoint=0; iJoint<mObjects[iObject]->getJoints().size(); iJoint++)
			for (unsigned int iMotor=0; iMotor<mObjects[iObject]->getJoints()[iJoint]->getNumMotors(); iMotor++)
			{
				CODEJointMotor* motor = mObjects[iObject]->getJoints()[iJoint]->getMotor(iMotor);
				if (motor != NULL)
					motor->update(stepTime);
			}
}

void CODESim::updateExternalForces()
{
	for (unsigned int iObject=0; iObject<mObjects.size(); iObject++)
		for (unsigned int iForce=0; iForce<mObjects[iObject]->getExternalForces().size(); iForce++)
			mObjects[iObject]->getExternalForces()[iForce]->update();
}

void CODESim::step()
{

	// Pre step function
	onPreStep();

	// Acquire sim access for the step + collision functions
	CODESimAccess simAccess(this);

	// Step function
	for (int iSS=0; iSS<mSubsamplingFactor; iSS++)
	{
		if (!mODEError)
		{
			try {
				// Do work that needs to be done before the simulation step:
				// - calculate collisions *before* sim step when geoms have been moved manually, outside ODE
				// - update all motors in all joints in all objects
				// - update external forces
				if (mCollisionsInvalid)
					calculateCollisions(&simAccess);

				updateJointMotors(getPartialStepTime());
				updateExternalForces();
				onPreSubStep();

				// Do the actual world step
				mWorld.step(getPartialStepTime());

				// Calculate collisions after the step function, so that contacts
				// between objects can be detected immediately after an integration step.
				calculateCollisions(&simAccess);

				onPostSubStep();
			} catch (...) {
				mODEError = true;
			}
		}
	}
	// Release sim access
	simAccess.release();

	// Post step function
	onPostStep();

	// Increment the time
	incrementTime();


	// Signal that we made a single step
	mEvtSingleStepDone = true;
}

void CODESim::calculateCollisions(CODESimAccess *simAccess)
{
	// Pre collide function
	onPreCollide();

	// Collide function
	mCollisionHandler.calculateCollisions(simAccess);
	mCollisionsInvalid = false;

	// Post collide function
	onPostCollide();
}

bool CODESim::shouldContinue()
{
	// If not in continuous mode (on 'pause'), wait for the next step
	if (!((bool)mEvtRunContinuously))
		mEvtShouldStep.wait();

	if ((bool)mEvtShouldStop)
		return false;
	else
		return true;
}

bool CODESim::start()
{
	bool result = false;
	// Lock start-stop mutex
	pthread_mutex_lock(&mStartStopMutex);

	// Start sim if not already started
	if (mWorkerThread != NULL)
		result = false;
	else
	{
		// Calculate collisions in the initial situation before taking the first step
		invalidateCollisions();

		// Set should-stop to false
		mEvtShouldStop	= false;
		// Create worker thread
		mWorkerThread = new CODESimWorkerThread(this);
		// Start worker thread
		mWorkerThread->start();

		result = true;
	}

	// Unlock start-stop mutex
	pthread_mutex_unlock(&mStartStopMutex);

	return result;
}

bool CODESim::stop()
{
	bool result = false;
	// Lock start-stop mutex
	pthread_mutex_lock(&mStartStopMutex);

	// Stop sim if not already stopped
	if (mWorkerThread == NULL)
		result = true;	// We are done! Return true
	else
	{
		// First, signal that we should stop
		mEvtShouldStop = true;
		// Then signal to take a step in order to wakeup the worker thread
		mEvtShouldStep = true;

		mWorkerThread->awaitTermination();
		delete mWorkerThread;
		mWorkerThread = NULL;

		// Make mEvtShouldStep false again to prevent the simworkerthread
		// from making further simulation steps when it is restarted again.
		mEvtShouldStep = false;
		result = true;
	}

	// Unlock start-stop mutex
	pthread_mutex_unlock(&mStartStopMutex);

	return result;
}

void CODESim::pause(bool shouldPause)
{
	mEvtRunContinuously = !shouldPause;
	if (!shouldPause)
		// Signal to make the first step
		mEvtShouldStep = true;
}

bool CODESim::singleStep()
{
	bool result=false;
	// Only make a new step if the previous one is done - check mEvtSingleStepDone
	// And only make a step if the sim is running
	if ((bool)mEvtSingleStepDone && (mWorkerThread != NULL))
	{
		// Reset mEvtSingleStepDone
		mEvtSingleStepDone = false;
		// Signal that the sim should take a new step
		mEvtShouldStep = true;
		// Wait for the single step to finish
		if (!mEvtSingleStepDone.wait(CONST_ODESIM_SINGLE_STEP_TIMEOUT))
		{
			// We timed out
			result = false;
			mLogErrorLn("CODESim::singleStep() timed out!");
		}
		else
		{
			result = !mODEError;
			//dbgprintf("[DEBUG] CODESim::singleStep() succeeded.\n");
		}
	}
	else
	{
		result = false;
		mLogErrorLn("CODESim::singleStep(): Please wait for the previous step to finish!");
	}
	return result;
}

void CODESim::invalidateCollisions(bool invalid)
{
	mCollisionsInvalid = invalid;
}

//************** Drawing *************//
void CODESim::getSimVisObjects(CSimVisObjectPtrArray* objects)
{
	// Draw contacts from the sim's collision handler, if desired
	if (mCollisionHandler.shouldDrawContacts())
	{
		for (unsigned int iContact=0; iContact<mCollisionHandler.getDrawingObjects().size(); iContact++)
		{
			objects->push_back(mCollisionHandler.getDrawingObjects()[iContact]);
		}
	}

	// For all objects..
	for (unsigned int iObject=0; iObject<mObjects.size(); iObject++)
	{
		CODEObject *pObject = mObjects[iObject];

		// Draw world body
		if (pObject->shouldDrawBodies())
			for (unsigned int iDrawingObject=0; iDrawingObject<mWorldBody.getDrawingObjects().size(); iDrawingObject++)
				objects->push_back(mWorldBody.getDrawingObjects()[iDrawingObject]);

		// For all other bodies in object..
		for (unsigned int iBody=0; iBody<pObject->getBodies().size(); iBody++)
		{
			CODEBody *pBody = pObject->getBodies()[iBody];
			// Draw bodies if desired
			if (pObject->shouldDrawBodies())
			{
				// For all drawing objects in body..
				for (unsigned int iDrawingObject=0; iDrawingObject<pBody->getDrawingObjects().size(); iDrawingObject++)
				{
					// Draw!
					objects->push_back(pBody->getDrawingObjects()[iDrawingObject]);
				}
			}

			// Draw Center of Mass if desired
			// TODO: fix drawing of CoMs
			//if (pObject->shouldDrawCoMs())
			//	drawCoM(pBody, 0.01f);
		}

		// For all joints in object..
		for (unsigned int iJoint=0; iJoint<pObject->getJoints().size(); iJoint++)
		{
			//CODEJoint *pJoint = pObject->getJoints()[iJoint];

			// Draw joints if desired
			if (pObject->shouldDrawJoints())
			{
				// TODO: implement drawing of joints
				/*
				// For all drawing objects in joint..
				for (unsigned int iDrawingObject=0; iDrawingObject<pJoint->getDrawingObjects().size(); iDrawingObject++)
				{
					// Draw!
					drawObject(pBody->getDrawingObjects()[iDrawingObject]);
				}
				*/
			}
		}

		// For all geoms in object..
		for (unsigned int iGeom=0; iGeom<pObject->getGeoms().size(); iGeom++)
		{
			CODEPlaceableGeom *pGeom = NULL;
			if (pObject->getGeoms()[iGeom]->isPlaceable())
				pGeom = (CODEPlaceableGeom*)pObject->getGeoms()[iGeom];

			// Draw geom if desired
			if (pObject->shouldDrawGeoms() && (pGeom != NULL))
			{
				// For all drawing objects in geom..
				for (unsigned int iDrawingObject=0; iDrawingObject < pGeom->getDrawingObjects().size(); iDrawingObject++)
				{
					// Draw!
					objects->push_back(pGeom->getDrawingObjects()[iDrawingObject]);
				}
			}
		}
	}
}


