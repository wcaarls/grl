/*
 * STGPolicy.h
 *
 *  Created on: Nov 25, 2008
 *      Author: Erik Schuitema
 */

#ifndef STGPOLICY_H_
#define STGPOLICY_H_

#include <STG.h>
#include <STGListener.h>
#include <Configuration.h>

/** One can install a policy (CSTGPolicy) into a policy player (CSTGPolicyPlayer).
 *  A policy can be of any type: a program that is fixed in time, a generated pattern,
 *  a state machine, etc.
 */

template<class STGStateType>
class CSTGPolicy
{
	protected:
		ISTGActuation	*mActuationInterface;
		uint64_t		mInstallTime;
		uint64_t		mResetTime;
		uint64_t		mResetMemoryTime;

	public:
		CSTGPolicy(ISTGActuation *actuationInterface):
			mActuationInterface(actuationInterface),
			mInstallTime(0),
			mResetTime(0),
			mResetMemoryTime(UINT64_MAX)
		{
		}
		virtual ~CSTGPolicy()	{}

		ISTGActuation*		getActuationInterface()	{return mActuationInterface;}

		// Override these to add functionality.

		// onInstall() will be called when this policy is installed in a policy player.
		// When this policy is installed in a policy player, onInstall() will be directly followed by a call to reset()
		virtual bool		onInstall(uint64_t absoluteTimeMicroseconds)
		{
			mInstallTime = absoluteTimeMicroseconds;
			// Set memory reset time on first install
			if (mResetMemoryTime == UINT64_MAX)
				mResetMemoryTime = absoluteTimeMicroseconds;
			return true;
		}
		virtual void		onUninstall(uint64_t absoluteTimeMicroseconds)	{}	// When replaced/removed in a policy player, this function will be called
		virtual bool		isDone(STGStateType* currentState, uint64_t absoluteTimeMicroseconds)	{return true;}	// Indicates whether the policy has finished
		// After calling reset(), the policy should be ready for a fresh start (but may keep its adaptive/learnnig memory).
		// When this policy is installed in a policy player, onInstall() will be directly followed by a call to reset()
		virtual void		reset(uint64_t absoluteTimeMicroseconds)		{mResetTime = absoluteTimeMicroseconds;}
		virtual void		resetMemory(uint64_t absoluteTimeMicroseconds)	{mResetMemoryTime = absoluteTimeMicroseconds;}	// For adaptive/learning controllers, this resets any internal adaptive/learning memory
		virtual bool		readConfig(const CConfigSection &configNode)	{return true;}
		virtual bool		save()											{return true;}
		virtual bool		save(std::ofstream& fileOutStream)				{return true;}
		uint64_t			getInstallTime()								{return mInstallTime;}	// Moment of installation
		uint64_t			getResetTime()									{return mResetTime;}	// Moment of last reset
		uint64_t			getResetMemoryTime()							{return mResetMemoryTime;}	// Moment of last memory reset


		// In init(), you can do pre-calculations and pre-installations of
		// your policy. You are also allowed to allocate memory, etc. because
		// init() should be called before the main real-time loop.
		virtual bool		init()											{return true;}

		// Version of init in which you can load data from a filestream
		virtual bool		init(std::ifstream& fileInStream)				{return true;}

		// executePolicy() calculates the desired actions / motor commands
		// and 'installs' - but doesn't execute - them; DO NOT call activateActions() inside this function!
		// You should call executePolicy() every time the policyplayer (=listener) receives a state.
		// In order to get notified of a new state, you have to call waitForNewState() yourself, e.g. in a (periodic) thread/task.
		// Return value: 0 if success, an error value otherwise
		virtual int			executePolicy(STGStateType* currentState, uint64_t absoluteTimeMicroseconds)=0;
};


template<class STGStateType>
class CSTGPolicyPlayer: public CSTGListener<STGStateType>
{
	protected:
		CSTGPolicy<STGStateType>*	mCurrentPolicy;
		ISTGActuation*				mActuationInterface;

	public:
		// The player needs an STG as well as an actuation interface.
		// Usually, one derives an STG that is also derived from ISTGActuation,
		// e.g. class CSTGLeoReal: public CSTGLeo, public ISTGLeoActuation.
		// Such an object can be passed for both parameters
		CSTGPolicyPlayer(CStateTransitionGenerator<STGStateType>* stg, ISTGActuation* stgActuation):
			CSTGListener<STGStateType>(stg),
			mCurrentPolicy(NULL),
			mActuationInterface(stgActuation)
		{
		}

		ISTGActuation* getActuationInterface()	{return mActuationInterface;}

		// installPolicy(): sets the current policy of this player.
		// Installing a new policy calls onUninstall() of the old policy
		// and onInstall() + reset() of the new policy, and returns the onInstall() result.
		virtual bool installPolicy(CSTGPolicy<STGStateType>* policy, uint64_t absoluteTimeMicroseconds)
		{
			if (mCurrentPolicy != NULL)
				mCurrentPolicy->onUninstall(absoluteTimeMicroseconds);
			mCurrentPolicy = policy;

			// You may install the NULL policy
			if (mCurrentPolicy == NULL)
				return true;

			if (mCurrentPolicy->onInstall(absoluteTimeMicroseconds))
			{
				// Call a reset when the policy is installed.
				mCurrentPolicy->reset(absoluteTimeMicroseconds);
				return true;
			}
			else
				return false;
		}

		// Return 0 if success, or an error value otherwise
		virtual int executePolicy(uint64_t absoluteTimeMicroseconds)
		{
			if (mCurrentPolicy != NULL)
				return mCurrentPolicy->executePolicy(this->getState(), absoluteTimeMicroseconds);
			else
				return 0;	// We can't fail in doing nothing?
		}

		bool isPolicyDone(STGStateType *currentState, uint64_t absoluteTimeMicroseconds)
		{
			if (mCurrentPolicy != NULL)
				return mCurrentPolicy->isDone(currentState, absoluteTimeMicroseconds);
			else
				return true;	// We are done if there is nothing to do
		}

		void resetPolicy(uint64_t absoluteTimeMicroseconds)
		{
			if (mCurrentPolicy != NULL)
				mCurrentPolicy->reset(absoluteTimeMicroseconds);
		}

		void resetPolicyMemory(uint64_t absoluteTimeMicroseconds)
		{
			if (mCurrentPolicy != NULL)
				mCurrentPolicy->resetMemory(absoluteTimeMicroseconds);
		}

		void savePolicy()
		{
			if (mCurrentPolicy != NULL)
				mCurrentPolicy->save();
		}

		CSTGPolicy<STGStateType>*	getCurrentPolicy()
		{
			return mCurrentPolicy;
		}

		// activateActions() actually sends the motor commands to the motors
		// A return value of 0 indicates success.
		int activateActions()
		{
			return mActuationInterface->activateActions(this->getState()->mStateID);	// Pass state ID of the most recently received state
		}
};

#endif /* STGPOLICY_H_ */
