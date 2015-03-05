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

#ifndef STGODESIM_H_
#define STGODESIM_H_

#include "ODESim.h"
#include <STG.h>
#include <Configuration.h>
#include <sstream>

template<class STGStateType>	// STGStateType must be derived from CSTGState
class CSTGODESim: public CStateTransitionGenerator<STGStateType>
{
	protected:
		CODESim				mSim;	// If desired, template the type of sim (now CODESim)
	public:
		// CSTGODESim():
		// The STG name is extended with the process ID to create a unique name per sim instance.
		CSTGODESim(const std::string& name):
			CStateTransitionGenerator<STGStateType>(name)
		{
		}

		uint64_t		getAbsTime()
		{
			return (uint64_t)(mSim.getTime()*1E6); // Absolute time in microseconds
		}

		CODESim*		getSim()
		{
			return &mSim;
		}

		virtual bool	init()
		{
			return mSim.init();
		}

		virtual bool	deinit()
		{
			return mSim.deinit();
		}

		virtual bool	isInitialized()
		{
			return mSim.isInitialized();
		}

		// Set noObjects to true if you don't want to read just all objects from XML but instead
		// want to explicitly configurate pre-created objects.
		virtual bool	readConfig(const CConfigSection &configSection, bool noObjects=false)
		{
			return mSim.readConfig(configSection, noObjects);
		}
};

#endif /* STGODESIM_H_ */
