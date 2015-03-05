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

#ifndef WAITEVENT_HPP_
#define WAITEVENT_HPP_

#include <pthread.h>
#include <assert.h>
#include <errno.h>

#ifdef _MSC_VER
	#include <win32_compat.h>
#endif

// WaitEvent: can be used to wait on a boolean condition
// Implemented using POSIX condition variables
// Only a single thread can wait for this event to become true.
class CWaitEvent
{
	protected:
		// POSIX condition variables should always be used in combination with a regular variable and a mutex
		// See for example https://computing.llnl.gov/tutorials/pthreads/#ConditionVariables

		bool			mManualReset;
		bool			mSignaled;	// The 'global' variable indicating the 'signaled' status. Protected by mMutex!
		pthread_mutex_t	mMutex;		// The mutex for mSignaled
		pthread_cond_t	mCond;		// The condition variable

	public:
		// Define constant for infinite waiting time
		static const int WAIT_TIME_INFINITE = -1;
		// Default parameters: a normal boolean (manual reset, initial state = false)
		CWaitEvent(bool bManualReset, bool bInitialState)
		{
			// Initialize signaled state
			mManualReset	= bManualReset;
			mSignaled		= bInitialState;
			// Initialize mutex and condition variable objects
			pthread_mutex_init(&mMutex, NULL);
			pthread_cond_init (&mCond, NULL);
		}

		~CWaitEvent()
		{
			pthread_mutex_destroy(&mMutex);
			pthread_cond_destroy(&mCond);
		}

		void	setManualReset(bool enabled)
		{
			mManualReset = enabled;
		}

		// Wait for the event to become signaled
		// Returns true if event actually became signaled; returns false on a timeout.
		inline bool wait()
		{
			return wait(CWaitEvent::WAIT_TIME_INFINITE);
		}

		inline bool wait(long int dwMilliseconds)
		{
			// 1) Lock mutex and check for mSignaled
			// 2) Wait for signal if needed. Note that the pthread_cond_wait routine
			//    will automatically and atomically unlock mutex while it waits.
			// 3) Unlock mutex again
			// Return whether the event actually became signaled
			bool becameSignaled;
			pthread_mutex_lock(&mMutex);

			// Wait only if not already signaled
			if (!mSignaled)
			{
				if (dwMilliseconds == CWaitEvent::WAIT_TIME_INFINITE)
				{
					// Implement the waiting in a while loop. The reason: the IEEE documentation:
					// "When using condition variables there is always a Boolean predicate involving shared variables associated with each condition wait that is true if the thread should proceed. Spurious wakeups from the pthread_cond_timedwait() or pthread_cond_wait() functions may occur. Since the return from pthread_cond_timedwait() or pthread_cond_wait() does not imply anything about the value of this predicate, the predicate should be re-evaluated upon such return."
					while (!mSignaled)
					{
						pthread_cond_wait(&mCond, &mMutex);
					}
				}
				else
				{
					timespec absWakeupTime;
				    clock_gettime(CLOCK_REALTIME, &absWakeupTime);
				    int waitSecs = dwMilliseconds / 1000;
				    absWakeupTime.tv_sec	+= waitSecs;
				    absWakeupTime.tv_nsec	+= (dwMilliseconds - waitSecs*1000) * 1000 * 1000;	// nanoseconds
					// Wrap nanoseconds
					if (absWakeupTime.tv_nsec >= 1E9)
					{
						absWakeupTime.tv_nsec -= (long)1E9;
						absWakeupTime.tv_sec++;
					}
					//printf("[DEBUG] CWaitEvent::wait(): tv_sec: %d, tv_nsec: %d\n", (int)absWakeupTime.tv_sec, (int)absWakeupTime.tv_nsec);
					// Implement the waiting in a while loop. The reason: the IEEE documentation on spurious wakeups from the waiting functions.
					int timedWaitResult = 0;
					while ((timedWaitResult != ETIMEDOUT) && (!mSignaled))
					{
						timedWaitResult = pthread_cond_timedwait(&mCond, &mMutex, &absWakeupTime);
						if (timedWaitResult == ETIMEDOUT)
						{
							// Avoid spurious wakeups
							timespec realWakeupTime;
							clock_gettime(CLOCK_REALTIME, &realWakeupTime);
							if (realWakeupTime.tv_sec < absWakeupTime.tv_sec ||
							    (realWakeupTime.tv_sec == absWakeupTime.tv_sec && realWakeupTime.tv_nsec < absWakeupTime.tv_nsec))
							{
								timedWaitResult = 0;
							}
						}
					}
					if ((timedWaitResult != 0) && (timedWaitResult != ETIMEDOUT))
						printf("[DEBUG] CWaitEvent::wait(): pthread_cond_timedwait() returned %d!\n", timedWaitResult);
				}
			}
			//else
			//	printf("[DEBUG] CWaitEvent::wait() does not need to wait because mSignaled is true.\n");

			// Return the real status of mSignaled before resetting it. When waiting timed out, mSignaled might be false.
			becameSignaled = mSignaled;
			if (!mManualReset)
				mSignaled = false;

			pthread_mutex_unlock(&mMutex);

			return becameSignaled;
		}

		// Assigning a bool for the signaled state
		inline bool operator=(const bool signaled)
		{
			if (signaled)
			{
				pthread_mutex_lock(&mMutex);
				// Set the signaled state to true
				mSignaled = true;
				// Signal any waiting thread that the condition became signaled
				pthread_cond_signal(&mCond);
				pthread_mutex_unlock(&mMutex);
			}
			else
			{
				pthread_mutex_lock(&mMutex);
				mSignaled = false;
				pthread_mutex_unlock(&mMutex);
			}
			return mSignaled;
		}

		// Requesting a bool
		inline operator bool ()
		{
			bool signaled;

			// Read our global variable by locking the mutex
			pthread_mutex_lock(&mMutex);
			signaled = mSignaled;
			pthread_mutex_unlock(&mMutex);

			return signaled;
		}

};


#endif /* WAITEVENT_HPP_ */
