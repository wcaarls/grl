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

#ifndef BASETHREAD_HPP_
#define BASETHREAD_HPP_


#include <string>
#include <Log2.h>


/**
 * The base of all platform specific thread implementations. Lightly modeled
 * after the Java Thread class.
 *
 * @author	G.A. vd. Hoorn		Initial implementation.
 */
class BaseThread
{
protected:
	CLog2	mThreadLog;

public:
	/**
	 * State machine guards against starting a thread after it's been stopped,
	 * etc.
	 */
	enum ThreadState
	{
		/// a thread is in state CREATED right after its been constructed.
		CREATED,
		/// a thread is in state RUNNING right after its @c start() method has been called.
		RUNNING,
		/// a thread is in state STOPPED after its @c run() method has returned and the thread has stopped executing.
		STOPPED
	};


	/**
	 * Enum is to be mapped onto specific thread implementations priority classes
	 * / numbers.
	 */
	enum ThreadPriority
	{
		IDLE,
		LOWER,
		LOW,
		NORMAL,
		HIGH,
		HIGHER,
		MAX
	};


private:
	/// a name for this thread (to be replaced by NamedObject when available).
	std::string _name;

	/// State of this thread (simple statemachine)
	ThreadState _tstate;

	/// Priority of this thread
	ThreadPriority _tpriority;

	// Boolean that the user can check in its run() function whether to stop or not
	bool _shouldStop;


public:
	BaseThread()
	:
		mThreadLog("thread"), _name("Thread0_needs_randomisation"), _tstate(BaseThread::CREATED), _tpriority(BaseThread::NORMAL), _shouldStop(false)
	{
		// empty
	}


	BaseThread(const std::string name, const ThreadPriority priority)
	:
		mThreadLog("thread"), _name(name), _tstate(BaseThread::CREATED), _tpriority(priority), _shouldStop(false)
	{
		// empty
	}


	virtual ~BaseThread()
	{
		// empty
	}


	const std::string& name() const
	{
		return _name;
	}


	ThreadState state() const
	{
		return _tstate;
	}


	ThreadPriority priority() const
	{
		// just return stored ThreadPriority
		return _tpriority;
	}


	virtual void priority(ThreadPriority priority)
	{
		// call conversion function for specific subclass
		_priority(priority);
	}


	/**
	 * @return	true IFF this thread has been created but not yet started.
	 */
	bool created() const
	{
		return (state() == BaseThread::CREATED);
	}


	/**
	 * @return	true IFF this thread is currently running.
	 */
	bool running() const
	{
		return (state() == BaseThread::RUNNING);
	}


	/**
	 * @return	true IFF this thread has stopped executing.
	 */
	bool stopped() const
	{
		return (state() == BaseThread::STOPPED);
	}

	bool shouldStop() const
	{
		return _shouldStop;
	}

	/**
	 * Starts the execution of this thread. In essence, this causes the @c run()
	 * method to be called.
	 */
	void start()
	{
		// Reset _shouldStop flag
		_shouldStop = false;
		// guard: only start thread if it is freshly created or completely stopped and closed
		if(created() || stopped())
		{
			// update state
			_trans_running();
			// call subclass implementor
			_start();
		}
		// TODO: perhaps do something in other cases? for now just ignore
	}

   /* Signals the USER to stop its thread */
	void stop()
	{
		_shouldStop = true;
	}

/**
 * Protected methods, to be used by subclasses.
 */
protected:
	/**
	 * Main run method, to be overridden by subclasses.
	 */
	virtual void run() = 0;

	/**
	 * (unconditionally) transitions the internal state machine to the
	 * running state.
	 */
	void _trans_running()
	{
		_tstate = BaseThread::RUNNING;
	}


	/**
	 * (unconditionally) transitions the internal state machine to the
	 * stopped state.
	 */
	void _trans_stopped()
	{
		_tstate = BaseThread::STOPPED;
	}


	/**
	 * @brief This method should start the implementations thread execution.
	 *
	 * Subclasses could either really create and start their thread instance
	 * in this method, or just start it.
	 * In every case, implementations should make sure the @c run() method is
	 * called after the thread has been started.
	 */
	virtual void _start() = 0;



	/**
	 * @brief This method should set the runtime scheduling priority of this
	 * thread to the implementations representation that is closest to the
	 * ThreadPriority given.
	 *
	 * @param	priority		The new thread priority.
	 */
	virtual void _priority(ThreadPriority priority) = 0;
};


#endif /* BASETHREAD_HPP_ */
