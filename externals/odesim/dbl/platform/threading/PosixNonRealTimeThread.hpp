/*
 *    Copyright (C) 2012 G.A. vd. Hoorn
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

#ifndef POSIXNONREALTIMETHREAD_HPP_
#define POSIXNONREALTIMETHREAD_HPP_

// the main posix thread header
#include <pthread.h>

#include <stdio.h>
#include <iostream>
#include <string.h>
// we are a non-real-time thread
#include <NonRealTimeThread.hpp>

/**
 * Class wraps a Posix non real-time userspace thread. Subclasses should
 * implement the @c run() method.
 *
 * @author	G.A. vd. Hoorn		Initial implementation.
 * @author	E.Schuitema			Crappification
 */
class PosixNonRealTimeThread : public NonRealTimeThread
{
public:
	/**
	 * Enum maps ThreadPriorities to Posix thread priorities. Not an exact
	 * mapping, I know.
	 *
	 * TODO: what are the proper priority numbers for this?
	 */
	enum PosixThreadPriority
	{
		IDLE   =   1,
		LOWER  =  16,
		LOW    =  32,
		NORMAL =  50,
		HIGH   =  66,
		HIGHER =  82,
		MAX    =  99
	};


private:
	/**
	 * The thread this class encapsulates.
	 */
	pthread_t 		_thread;
	bool			_thread_created;


	/**
	 * Attributes of this thread.
	 */
	pthread_attr_t	_thread_attr;



public:
	PosixNonRealTimeThread()
	:
		NonRealTimeThread(),
		_thread_created(false)
	{
		// initialise attributes
		pthread_attr_init(&_thread_attr);
	}

	PosixNonRealTimeThread(const std::string name, const ThreadPriority priority)
	:
		NonRealTimeThread(name, priority),
		_thread_created(false)
	{
		// initialise attributes
		pthread_attr_init(&_thread_attr);
	}

	/**
	 * awaitTermination() waits until the thread has finished.
	 * Return value: the return value of __arg_run()
	 */
	virtual void* awaitTermination()
	{
		// std::cout << "finish" << std::endl;
		// Create status pointer for pthread_join() (also see pthread_exit())
		void* pstatus = NULL;

		// wait for task to end, if the thread is running
		if (running())
			pthread_join(_thread, &pstatus);

		// we know for sure that the task has stopped here
		_trans_stopped();
		return pstatus;
	}

	virtual ~PosixNonRealTimeThread()
	{
		awaitTermination();

		// free resources
		pthread_attr_destroy(&_thread_attr);
		if (_thread_created)
			pthread_detach(_thread);
	}


protected:
	/**
	 * Specific Posix start implementation.
	 *
	 * Steps:
	 *  - convert priority into Posix priority
	 *  - create thread
	 *  - start it
	 *
	 */
	void _start()
	{
		// get prio converted
		const int t_prio = _taskPrioToPosixPrio(priority());
		// set flags for posix thread
		const int t_flags = PTHREAD_CREATE_JOINABLE;

		// initialise attributes
		pthread_attr_init(&_thread_attr);
		// create joinable
		pthread_attr_setdetachstate(&_thread_attr, t_flags);

		// now create thread
		int res = pthread_create(&_thread, &_thread_attr, &__arg_run, static_cast<void*>(this));
		// check res and throw some exceptions for a change
		if (res != 0)
		{
			_thread_created = false;
			logErrorLn(mThreadLog, "pthread_create for thread \"" << name() << "\" failed with code " << res);
		}
		else
		{
			_thread_created = true;
			logDebugLn(mThreadLog, "Successfully created thread \"" << name() << "\"");
		}

		// get scheduler params for the thread
		int policy;
		sched_param param;
		res = pthread_getschedparam(_thread, &policy, &param);
		logCrawlLn(mThreadLog, "pthread_getschedparam result: policy = " << policy << ", param = " << param.sched_priority);

		// set priority
		param.sched_priority = t_prio;

		// set scheduler params back for the thread
	    res = pthread_setschedparam(_thread, policy, &param);
		// check res and throw some exceptions for a change
		if (res != 0)
		{
			int minPrior = sched_get_priority_min(policy);
			int maxPrior = sched_get_priority_max(policy);
			if (minPrior != maxPrior)	// Only warn when priorities are supported at all
				logWarningLn(mThreadLog, "pthread_setschedparam(" << name() << ") returned \"" << strerror(res) << "\" while trying to set priority to " << param.sched_priority << " (should be between " << minPrior << " and " << maxPrior << ")");
		}
		else
			logCrawlLn(mThreadLog, "pthread_setschedparam for thread \"" << name() << "\" returned " << res);
	}

	void _priority(ThreadPriority priority)
	{
		// TODO: implement this; posix threads can change priority
	}


	/**
	 * Converts ThreadPriority priority into Posix priority. This isn't an
	 * exact science (as you can see).
	 *
	 * @param	priority		The priority to convert (or map).
	 * @return					A Posix priority number.
	 */
	int _taskPrioToPosixPrio(const ThreadPriority priority)
	{
		switch(priority)
		{
		case BaseThread::IDLE:
			return PosixNonRealTimeThread::IDLE;

		case BaseThread::LOWER:
			return PosixNonRealTimeThread::LOWER;

		case BaseThread::LOW:
			return PosixNonRealTimeThread::LOW;

		case BaseThread::NORMAL:
			return PosixNonRealTimeThread::NORMAL;

		case BaseThread::HIGH:
			return PosixNonRealTimeThread::HIGH;

		case BaseThread::HIGHER:
			return PosixNonRealTimeThread::IDLE;

		case BaseThread::MAX:
			return PosixNonRealTimeThread::MAX;

		default:
			return PosixNonRealTimeThread::NORMAL;
		}
	}


private:
	/**
	 * @brief Function calls the run method of the PosixNonRealTimeThread it was
	 * handed.
	 *
	 * This is needed since the Posix thread needs a c function pointer, while
	 * the @c run() method is a method pointer on a specific instance. Using
	 * this construct we can actually give the Posix thread task its function
	 * pointer which calls our method 'pointer'.
	 *
	 * @param	arg		Pointer which is assumed to be the PosixNonRealTimeThread to run.
	 */
	static void* __arg_run(void* arg)
	{
		// cast
		PosixNonRealTimeThread* t = static_cast<PosixNonRealTimeThread*>(arg);

		// just call run method
		t->run();

		// update task state
		t->_trans_stopped();

		// could call pthread_exit(), but this is implied by returning from
		// this function.
		return NULL;
	}
};


#endif /* POSIXNONREALTIMETHREAD_HPP_ */
