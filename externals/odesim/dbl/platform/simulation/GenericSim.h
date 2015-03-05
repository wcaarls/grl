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

#ifndef GENERICSIM_H_
#define GENERICSIM_H_

#include <stdio.h>

class CGenericSimAccess;

// Generic simulator class
class CGenericSim
{
	private:
		bool				mInitialized;
	protected:
		pthread_mutex_t		mAccessLockMutex;
		void				setInitialized(bool inited)	{mInitialized = inited;}

	public:
		CGenericSim():
			mInitialized(false)
		{
			pthread_mutex_init(&mAccessLockMutex, NULL);
		}

		virtual ~CGenericSim()
		{
			pthread_mutex_destroy(&mAccessLockMutex);
		}

		// Acquire sim access to acces the 'internals' of the sim.
		// This mechanism was introduced to avoid requesting objects during, e.g. drawing, while the sim is making a step and therefore changing the objects.
		// The CGenericSimAccess* parameter is historic and lost his purpose, but may come in handy in the future in an improved locking structure
		virtual bool	lock(CGenericSimAccess* simAccess)
		{
			if (simAccess != NULL)
			{
				pthread_mutex_lock(&mAccessLockMutex);
				return true;
			}
			else
				return false;
		}

		virtual void	unlock(CGenericSimAccess* simAccess)
		{
			if (simAccess != NULL)
			{
				pthread_mutex_unlock(&mAccessLockMutex);
			}
		}

		// Initialization
		virtual bool	init()			{setInitialized(true); return true;}
		virtual bool	deinit()		{setInitialized(false); return true;}
		inline bool		isInitialized() { return mInitialized; }

		// Time functions
		virtual double	getTime(const int timerIndex=0)	{return 0;}
};

// Access class to a generic simulator
class CGenericSimAccess
{
	protected:
		CGenericSim*	mpSim;	// If and only if mpSim is not NULL, access is granted.
	public:
		CGenericSimAccess():
			mpSim(NULL)
		{
		}

		CGenericSimAccess(CGenericSim *sim)
		{
			if (!acquire(sim))
				printf("[ERROR] Could not acquire a lock on the simulator!\n");
		}
		~CGenericSimAccess()
		{
			release();
		}

		// The preferred way is to use the constructor with 'sim' parameter
		// instead of acquire().
		bool	acquire(CGenericSim *sim)
		{
			if (sim->lock(this))
			{
				mpSim = sim;
				return true;
			}
			else
				return false;
		}

		void	release()
		{
			if (mpSim)
			{
				mpSim->unlock(this);
				mpSim = NULL;
			}
		}
};

#endif /* GENERICSIM_H_ */
