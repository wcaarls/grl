/*
 *    Implementation of State Transition Generator listener class
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

#ifndef STGLISTENER_H_
#define STGLISTENER_H_

#include <mqueue.h>
#include <STG.h>

/** Class that is able to receive messages from a State Transition Generator
*  When deriving your own listener class from this class, be sure to make use of
*  the functions startListening() and stopListening(), otherwise waitForNewState() has no effect!
*/
template<class STGStateType>
class CSTGListener
{
	private:
		CSTGInQueue<STGStateType>					mInQueue;
		CStateTransitionGenerator<STGStateType>		*mSTG;

	public:
		CSTGListener(CStateTransitionGenerator<STGStateType>* stg)
		{
			mSTG = stg;
		}
		virtual ~CSTGListener()
		{

		}

		inline bool waitForNewState()
		{
			return mInQueue.waitForNewState();
		}

		inline CStateTransitionGenerator<STGStateType> *getSTG()
		{
			return mSTG;
		}

		inline STGStateType* getState()
		{
			return mInQueue.getState();
		}

		inline bool startListening(ESTGReceiveMode receiveMode, int frequencyDivider, int queueLength, const char *nameExtension)
		{
			std::string stgQueueName = mSTG->createSubscription(receiveMode, frequencyDivider, queueLength, nameExtension);
			if (stgQueueName.empty())
				return false;

			return mInQueue.open(stgQueueName);
		}

		inline bool stopListening()
		{
			int retval = mInQueue.close();
			mSTG->stopSubscription(mInQueue.getName());
			return retval;
		}

};



#endif /* STGLISTENER_H_ */
