/*
 *    Implementation of State Transition Generator base class
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

#ifndef STG_H_
#define STG_H_

/**A State Transition Generator (STG) will basically do two things:
 * 1) Produce periodic state information based on either measurements or simulation
 * 2) Notify connected listeners of a state transition and provide the new state information
 *
 * In an STG, the source of the state transitions (i.e. simulation or measured) is abstracted away
 *
 * The interface is POSIX compatible and is based on POSIX message queues,
 * therefore it should compile on Xenomai real-time systems using the POSIX skin as well as non-real-time systems.
 *
 *
 * IMPORTANT NOTE: keep CStateTransitionGenerator and ISTGActuation
 * separated for as long as possible when deriving classes. Eventually, both
 * classes can be merged. But when separated, one can create an STG for several
 * simulation packages, for example, CSTGODESim (for Open Dynamics Engine), which
 * is then used in class CSTGLeoSim: public CSTGSimODE, public ISTGLeoActuation;
 *
 */


#include <mqueue.h>
#include <string>
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <vector>
#include <errno.h>
#include <unistd.h>

#ifdef _WIN32
	#include <process.h>
	#define getpid _getpid
#endif

#ifndef UINT64_MAX
	# define UINT64_MAX		(18446744073709551615ULL)
#endif


#include <Log2.h>

#define STG_MAX_OUT_QUEUES		10
#define STG_MAX_NAME_LENGTH		255

class CSTGLoggable
{
  protected:
    CLog2   mStgLog;
  public:
    CSTGLoggable(): mStgLog("stg") {}
};

class CSTGState
{
	public:
		virtual ~CSTGState()	{}
		uint64_t		mStateID;	// State identifier: must be incremented by one between successive states.
		int				mLastError; // error passing facility

		virtual void	clear()	// Clears everything inside the state (called by user)
		{
			mStateID		= 0;
			mLastError		= 0;
			//logCrawlLn(CLog2("stg"), "Cleared CSTGState");
		}

		// Right now, we use the same mStateID constant to indicate that the state is either invalid or the last one (terminal)
		inline bool isTerminal()	{ return (mStateID == UINT64_MAX); }
		inline void	setTerminal()	{ mStateID = UINT64_MAX; }

		inline bool isValid()		{ return (mStateID != UINT64_MAX); }
		inline void invalidate()	{ mStateID = UINT64_MAX; }

		// Logging functions
		// It would be more logical to make toTextHeader() static, but then it cannot be virtual
		virtual const std::string	toTextHeader()	const {return std::string("");}
		virtual std::string			toText()		const {return std::string("");}

};


// Define a joint index transform function type: transforms a joint index into a new one, e.g., to mirror a policy
typedef int(*CSTGJointTransformFunc)(const int jointIndex);

enum ESTGActuationMode
{
	amOff,
	// amIndividual means: no common actuation mode. Actuators are controlled on an individual command basis.
	amIndividual,
	// The modes below are modes in which all motors receive their command every time step.
	amPosition,
	amSpeed,
	amTorque,
	amForce,	// Linear actuators
	amVoltage,
	amCurrent,
	amPWM,
	amIntAction,
	amTorqueInit,
	amIdle
};

/// Actuation interface for the STG
// Return codes are integer and successful function calls should return zero (0).
class ISTGActuation
{
	public:
		ISTGActuation()				{}
		virtual ~ISTGActuation()	{}
/**		 setJointPosition(), setJointSpeed(), setJointTorque(), setJointCurrent() and setJointPWM()
*		 must be supported by the underlying motor, since they are direct motor commands.
*		 A servo motor will probably support position, speed and perhaps torque,
*		 while a DC motor will probably only support setJointCurrent() or setJointPWM().
*		 *ANY* indirect control on top of the lowest level motor commands must be done outside the STG.
*
*		 Set the right mode first before calling SetJoint*()
*/
		virtual int					setActuationMode(ESTGActuationMode actuationMode)	{return 0;}
		ESTGActuationMode			getActuationModeByName(const std::string& actuationMode)
		{
			if (actuationMode.compare("off") == 0)
				return amOff;
			else if (actuationMode.compare("individual") == 0)
				return amIndividual;
			else if (actuationMode.compare("position") == 0)
				return amPosition;
			else if (actuationMode.compare("speed") == 0)
				return amSpeed;
			else if (actuationMode.compare("torque") == 0)
				return amTorque;
			else if (actuationMode.compare("force") == 0)
				return amForce;
			else if (actuationMode.compare("voltage") == 0)
				return amVoltage;
			else if (actuationMode.compare("current") == 0)
				return amCurrent;
			else if (actuationMode.compare("PWM") == 0)
				return amPWM;
			else if (actuationMode.compare("intaction") == 0)
				return amIntAction;
			else if (actuationMode.compare("torque_init") == 0)
				return amTorqueInit;
			else	// Default
				return amOff;
		}

		virtual void				setJointPosition(int jointIndex, double position, double speed)	{}	// Position in [rad]. Speed in [rad/s] and positive. Negative speed means: maximum speed.
		virtual void				setJointSpeed(int jointIndex, double speed)			{}	// Speed in [rad/s]
		virtual void				setJointTorque(int jointIndex, double torque)		{}	// Torque in [Nm]
		virtual void				setJointForce(int jointIndex, double torque)		{}	// Force in [N] for linear actuators
		virtual void				setJointVoltage(int jointIndex, double voltage)		{}	// Voltage in [V]
		virtual void				setJointCurrent(int jointIndex, double current)		{}	// Current in [A]
		virtual void				setJointPWM(int jointIndex, double PWM)				{}	// PWM in [%]
		virtual void 				setSeaPid(int jointIndex, double position, double speed, double torque, double pPos, double dPos) {}
		virtual void				setJointIntAction(int jointIndex, int action)		{}
		virtual void				setJointLock(int jointIndex, bool action)			{}
		virtual void				setLed(int ledIndex, bool action)					{}
		virtual void				setStatusStr(const std::string& status)				{}
		virtual void				setMotorPower(bool action)							{}
		virtual void				setParameters()										{}
        virtual void                setJointAction(int jointIndex, double action)       {}



		// It's wise to implement the physical properties of the motors, as well as their name
		virtual const std::string	getJointName(int jointIndex)						{return "";}
		virtual int					getJointIndexByName(const std::string& jointName)	{return 0;}

		virtual double				getJointMaxSpeed(int jointIndex)					{return 0;}
		virtual double				getJointMaxTorque(int jointIndex)					{return 0;}
		virtual double				getJointMaxVoltage(int jointIndex)					{return 0;}
		virtual double				getJointMaxCurrent(int jointIndex)					{return 0;}
		virtual double				getJointMaxPWM(int jointIndex)						{return 0;}
		virtual double				getJointMinPosition(int jointIndex)					{return 0;}
		virtual double				getJointMaxPosition(int jointIndex)					{return 0;}
		virtual double				getJointTemperature(int jointIndex)					{return 0;}
		// Call activateActions() at a moment of your choosing to put the new control actions into effect.
		// To be able to determine to which state this is a response, a stateID is passed
		virtual int					activateActions(const uint64_t& stateID)			{return 0;}

		// The actuation interface also provides an estimate of the computational delay between state measurements and control action execution
		// The computational delay is defined as the ratio of the delay time and the sample time, and can exceed 1.
		virtual double				getComputationalDelayRatio()						{return 0.0;}
};

enum ESTGReceiveMode
{
	stgReceiveAll,
	stgReceiveLatest
};

class CSTGOutQueue: public CSTGLoggable
{
	protected:
		std::string		mName;
		mqd_t			mDescriptor;
		ESTGReceiveMode	mMode;
		int				mQueueLength;
		int				mMessageLength;
		int				mFreqDivider;	// Frequency divider

		void			flushQueue()
		{
#if 1
			char	*tempState;
			tempState = new char[mMessageLength];
#else
			char tempState[mMessageLength];
#endif
			ssize_t queueResult = 0;
			while (queueResult != -1)
			{
				// mq_receive and mq_send may return EINTR when interrupted by a signal handler.
				while ((queueResult = mq_receive(mDescriptor, tempState, mMessageLength, NULL)) == -1 && errno == EINTR);
				//if (queueResult != -1)
					//mLogDebugLn("Discarded unread msg with length " << queueResult);
			}
#if 1
			delete tempState;
#endif
		}
	public:
		CSTGOutQueue()
		{
			mDescriptor	= (mqd_t)-1;
		}

		const std::string&	getName()
		{
			return mName;
		}

		int				getFreqDivider()
		{
			return mFreqDivider;
		}

		bool			open(const std::string& name, ESTGReceiveMode mode, int frequencyDivider, int messageLength, int queueLength)
		{
			// Fill member fields
			mName			= name;
			mMode			= mode;
			mFreqDivider	= frequencyDivider;
			mQueueLength	= queueLength;
			mMessageLength	= messageLength;

			// Create the output queue in non-blocking mode
			mq_attr queueOutAttr;
			queueOutAttr.mq_flags	= 0;
			queueOutAttr.mq_maxmsg	= queueLength;      /* Max. # of messages on queue */
			queueOutAttr.mq_msgsize	= messageLength;     /* Max. message size (bytes) */
			queueOutAttr.mq_curmsgs	= 0;     /* # of messages currently in queue */

			if (mq_unlink(mName.c_str()) == 0) //TODO: reconsider that killing this queue may not be desirable!
					logWarningLn(mStgLog, "Killed and reopened STG queue \"" << mName << "\". If your program recently quit prematurely (i.e. was killed), ignore this message. Otherwise, this indicates a naming conflict!");

			// Create a non-blocking queue and fail if it already exists (exclusive open)
			mDescriptor = mq_open(mName.c_str(), O_RDWR | O_CREAT | O_EXCL | O_NONBLOCK, S_IRWXU, &queueOutAttr);
			if (mDescriptor == (mqd_t)(-1))
				// fail
			{
				logErrorLn(mStgLog, "Could not create STG output queue \"" << mName << "\"! Error " << errno << ": " << strerror(errno));
				return false;
			}
			else
				logNoticeLn(mStgLog, "STG output queue \"" << mName << "\" opened!");
			return true;
		}

		bool			close()
		{
			if (mq_close(mDescriptor) != 0)
			{
				logErrorLn(mStgLog, "Could not close STG output queue \"" << mName << "\"!");
				return false;
			}
			if (mq_unlink(mName.c_str()) != 0)
			{
				logErrorLn(mStgLog, "Could not unlink STG output queue \"" << mName << "\"!");
				return false;
			}
			return true;
		}

		bool			send(const char* msg)
		{
			if (mMode == stgReceiveLatest)
				flushQueue();
			// Send the message!
			//int result = mq_send(mDescriptor, msg, mMessageLength,  MQ_PRIO_MAX-1);
			int result;
			// mq_receive and mq_send may return EINTR when interrupted by a signal handler.
			while ((result = mq_send(mDescriptor, msg, mMessageLength,  1)) == -1 && (errno == EAGAIN || errno == EINTR));

			return result == 0;
		}

        inline bool     isOpen()
        {
            return (mDescriptor != (mqd_t)(-1));
        }
};

/// Standard STG input queue template. Queue is closed in the destructor automatically.
/// When waitForNewState() returns, the newly received state is copied to the mState member variable.
template<class STGStateType>
class CSTGInQueue: public CSTGLoggable
{
	protected:
		std::string		mName;
		mqd_t			mDescriptor;
		STGStateType	mState;
	public:
		CSTGInQueue()
		{
			mDescriptor = (mqd_t)-1;	// = invalid file descriptor
		}

		~CSTGInQueue()
		{
			close();
		}

		const std::string&	getName()
		{
			return mName;
		}

		bool			open(const std::string& name)
		{
			// Fill member fields
			mName = name;

			// Create the input queue in blocking mode
			mDescriptor = mq_open(mName.c_str(), O_RDONLY);
			if (mDescriptor == (mqd_t)(-1))
				// fail
			{
				logErrorLn(mStgLog, "Could not create STG input queue \"" << mName << "\"!");
				return false;
			}
			else
				logNoticeLn(mStgLog, "STG input queue \"" << mName << "\" opened!");
			return true;
		}

		bool			close()
		{
			if (isOpen())
			{
				if (mq_close(mDescriptor) != 0)
				{
					logErrorLn(mStgLog, "Could not close STG input queue \"" << mName << "\"!");
					return false;
				}
				else
				{
					mDescriptor = (mqd_t)(-1);
					return true;
				}
			}
			else
				return true;
		}

		bool 			waitForNewState()
		{
			if (!isOpen())
				return false;

			unsigned msgPriority;
			// mq_receive and mq_send may return EINTR when interrupted by a signal handler.
			int msgResult;
			while ((msgResult = mq_receive(mDescriptor, (char*)&mState, sizeof(STGStateType), &msgPriority)) == -1 && errno == EINTR);
			if ((msgResult != -1) && (!mState.isTerminal()))
			{
				// We received a message from the STG!
				return true;
			}
			else
			{
				// This state was either bad or terminal
				if (mState.isTerminal())
					logDebugLn(mStgLog, "STG input queue \"" << mName << "\" received a terminal state!");
				else
					logErrorLn(mStgLog, "STG input queue \"" << mName << "\"'s mq_receive() returned " << msgResult << ": errno " << errno << " (" << strerror(errno) << ")");
				return false;
			}
		}

		// mState is only overwritten in waitForNewState().
		// Typically, the user calls waitForNewState() *after* it is done using the state.
		// Therefore, we can return a pointer without having to worry about thread safety.
		STGStateType	*getState()
		{
			return &mState;
		}

		inline bool		isOpen()
		{
			return (mDescriptor != (mqd_t)(-1));
		}

};

template<class STGStateType>	// STGStateType must be derived from CSTGState
class CStateTransitionGenerator: public CSTGLoggable
{
	private:
		std::string			getPIDStr()
		{
			std::stringstream pidstr;
			pidstr << "[" << getpid() << "]";
			return pidstr.str();
		}

	protected:
		std::string			mName;
		std::vector<CSTGOutQueue*>	mOutQueues;

		STGStateType		mState;	// Current state; to be sent to all subscribed mOutQueues

		int					getOutQueueIndex(const std::string& queueName)
		{
			for (unsigned int iQ=0; iQ<mOutQueues.size(); iQ++)
			{
				if (mOutQueues[iQ]->getName() == queueName)
					return iQ;
			}
			return -1;
		}

		void				clearOutQueues()
		{
			for (unsigned int i=0; i<mOutQueues.size(); i++)
			{
				mOutQueues[i]->close();
				delete mOutQueues[i];
			}
			mOutQueues.clear();
		}

		bool				broadcast()
		{
			bool result = true;
			// Broadcast on all out-queues at their desired rate (which means, every "getFreqDivider()"th sample)
			for (unsigned int iQ=0; iQ<mOutQueues.size(); iQ++)
				if (mState.mStateID % mOutQueues[iQ]->getFreqDivider() == 0)
					result &= mOutQueues[iQ]->send((char*)&mState);

			return result;
		}

		// broadcastTerminalState() sends a terminal state that will stop all listeners (waitForNewState() will return false)
		bool				broadcastTerminalState()
		{
			bool result = true;
			STGStateType termState;
			termState.setTerminal();
			// Broadcast on all out-queues
			for (unsigned int iQ=0; iQ<mOutQueues.size(); iQ++)
				result &= mOutQueues[iQ]->send((char*)&termState);

			return result;
		}

	public:
		CStateTransitionGenerator(const std::string& name):
			mName(name + getPIDStr())
		{
			// If the following code generates a compiler error,
			// you didn't derive STGStateType from CSTGState!
			STGStateType* pDerived = NULL;
			CSTGState* pBase = static_cast<CSTGState*>(pDerived);
			(void) pBase;	// To avoid "unused variable" warning
		}

		virtual ~CStateTransitionGenerator()
		{
			// Delete all output queues
			clearOutQueues();
		}

		virtual bool		init()			{return 0;}
		virtual bool		deinit()		{return 0;}

		virtual bool		start()			{return 0;}
		virtual bool		stop()			{return 0;}

		virtual uint64_t	getAbsTime()	{return 0;}	// Absolute time in microseconds

		// Use createSubscription to request/create a queue that sends you state information
		// frequencyDivider = X: you will receive every Xth sample in your queue
		// Return value: the queue name that you can use in a call to CSTGInQueue::Open(), or empty string if failed.
		// Subscriptions are unique. Function will fail if a subscription with that name already exists.
		// Subscriptions are system-global!! (visible beyond process boundary)
		std::string	createSubscription(ESTGReceiveMode receiveMode, int frequencyDivider, int queueLength, const char *nameExtension)
		{
			// Build queue name
			std::string outQueueName = "/" + mName + nameExtension;

			int queueIndex = getOutQueueIndex(outQueueName);
			if (queueIndex >= 0)
			{
				logErrorLn(mStgLog, "Trying to open already opened STG out-queue named \"" << outQueueName << "\"!");
				return "";
			}
			else
			{
				mOutQueues.push_back(new CSTGOutQueue);
				if (mOutQueues.back()->open(outQueueName, receiveMode, frequencyDivider, sizeof(STGStateType), queueLength))
				{
					//const char* result = mOutQueues[queueIndex]->getName();
					//mNumOutQueues++;
					//return result;
					return mOutQueues.back()->getName();
				}
				else
				{
					// Remove last element, because we failed
					delete mOutQueues.back();
					mOutQueues.pop_back();
					return "";
				}
			}
		}

		void		stopSubscription(const std::string& queueName)
		{
			int queueIndex = getOutQueueIndex(queueName);
			if (queueIndex >= 0)
			{
				mOutQueues[queueIndex]->close();
				delete mOutQueues[queueIndex];
				mOutQueues.erase(mOutQueues.begin() + queueIndex);
				logDebugLn(mStgLog, "Removed STG subscription queue \"" << queueName << "\"");
			}
		}
};

/* Test code to see if the template parameter check within CStateTransitionGenerator' constructor works
class CNonSTGState//: public CSTGState
{};

class CSTGTester: public CStateTransitionGenerator<CNonSTGState>
{
	public:
		CSTGTester(const std::string& name): CStateTransitionGenerator<CNonSTGState>(name)	{}
};
*/

#endif /* STG_H_ */
