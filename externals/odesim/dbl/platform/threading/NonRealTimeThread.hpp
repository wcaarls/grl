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

#ifndef NONREALTIMETHREAD_HPP_
#define NONREALTIMETHREAD_HPP_


#include <BaseThread.hpp>


/**
 * Class does not add any functionality but is only a 'decorator' in the sense
 * that if an 'end-user' requires a non-real-time thread implementation but
 * doesn't care which one he can ask for a @c NonRealTimeThread instead of a
 * @c BaseThread.
 *
 * @author	G.A. vd. Hoorn		Initial implementation.
 */
class NonRealTimeThread : public BaseThread
{
public:
	NonRealTimeThread()
	:
		BaseThread()
	{
		// empty constructor
	}


	NonRealTimeThread(const std::string name, const ThreadPriority priority)
	:
		BaseThread(name, priority)
	{
		// empty
	}


	virtual ~NonRealTimeThread()
	{
		// empty
	}
};


#endif /* NONREALTIMETHREAD_HPP_ */
