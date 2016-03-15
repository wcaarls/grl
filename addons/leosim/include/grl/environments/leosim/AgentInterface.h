/*
 * AgentInterface.h
 *
 *  Created on: Oct 15, 2009
 *      Author: Erik Schuitema
 */

#ifndef AGENTINTERFACE_H_
#define AGENTINTERFACE_H_

//#include <string>
#include "precisions.h"
//#include <randomc.h>
#include <fstream>
#include <cstring>
#include <sstream>
#include <vector>
#include <math.h>

#define AGENT_MAX_NUM_STATEVARS		20	// Maximum number of state dimensions
#define AGENT_MAX_NUM_ACTIONVARS	10	// Maximum number of action dimensions

class CAgentStateVarDescriptor
{
	public:
		_StatePrecision mScale;
		_StatePrecision mOffset;

		CAgentStateVarDescriptor():	mScale(1.0), mOffset(0.0) {}
		void	set(_StatePrecision scale=1.0, _StatePrecision offset=0.0)	{mScale=scale; mOffset=offset;}
};

class CAgentState
{
	protected:
		// The State variables and functions;
		int							mNumStateVars;
		bool						mWrapState;
		// These arrays are of fixed size to avoid malloc/new-delete in realtime applications
		_StatePrecision				mState[AGENT_MAX_NUM_STATEVARS];
		int							mStateWrapData[AGENT_MAX_NUM_STATEVARS];	// Defines after how many *transformed* units (see descriptors) the state should wrap

	public:
		CAgentStateVarDescriptor	mDescriptors[AGENT_MAX_NUM_STATEVARS];

		// Constructor
		CAgentState(int iNumStateVars=1) : mNumStateVars(0)
		{
			setNumStateVars(iNumStateVars);
			mWrapState = false;
		}

		CAgentState(const CAgentState& state)
		{
			mNumStateVars	= state.mNumStateVars;
			mWrapState		= state.mWrapState;
			memcpy(mState,			state.mState,			mNumStateVars*sizeof(mState[0]));
			memcpy(mStateWrapData,	state.mStateWrapData,	mNumStateVars*sizeof(mState[0]));
			// Copy descriptors
			for (int i=0; i<mNumStateVars; i++)
				mDescriptors[i] = state.mDescriptors[i];
		}

		// Init functions
		void setNumStateVars(int iNumStateVars)
		{
			if (mNumStateVars != iNumStateVars)
			{
				mNumStateVars = iNumStateVars;
				memset(mState, 0, mNumStateVars*sizeof(mState[0]));
				memset(mStateWrapData, 0, mNumStateVars*sizeof(mStateWrapData[0]));
			}
		}

		int getNumStateVars() const
		{
			return mNumStateVars;
		}

		inline _StatePrecision getStateVar(const int index) const
		{
			return mState[index];
		}

		inline void getScaledState(_StatePrecision *scaledState) const
		{
			for (int i=0; i<mNumStateVars; i++)
				scaledState[i] = mDescriptors[i].mScale*(mState[i]-mDescriptors[i].mOffset);
		}

		inline _StatePrecision&			operator[](unsigned int index)			{return mState[index];}
		inline const _StatePrecision&	operator[](unsigned int index)	const	{return mState[index];}

		inline void setStateVar(const int index, _StatePrecision value)
		{
			mState[index] = value;
		}

		inline void setStateVarWrap(const int index, int value)
		{
			mStateWrapData[index] = value;
		}

		inline const _StatePrecision* getState() const
		{
			return mState;
		}

		inline _StatePrecision* getState()
		{
			return mState;
		}

		void enableWrapping(bool enabled)
		{
			mWrapState = enabled;
		}

		int* getStateWrapData()
		{
			return mStateWrapData;
		}

		const int* getStateWrapData() const
		{
			if (mWrapState)
				return mStateWrapData;
			else
				return NULL;
		}

		bool wrapState() const
		{
			return mWrapState;
		}

		int getStateWrapDataVar(const int index)
		{
			return mStateWrapData[index];
		}

		// store(): fills the provided parameter with values from mState
		inline void		store(_StatePrecision *valuesOut) const
		{
			memcpy(valuesOut, mState, mNumStateVars*sizeof(_StatePrecision));
		}

		// restore(): fills mAction with the values provided
		inline void		restore(_StatePrecision *valuesIn)
		{
			memcpy(mState, valuesIn, mNumStateVars*sizeof(_StatePrecision));
		}

		bool	save(std::ofstream &fileOutStream)
		{
			fileOutStream.write((char*)&mNumStateVars,	sizeof(mNumStateVars));
			fileOutStream.write((char*)&mWrapState,		sizeof(mWrapState));
			fileOutStream.write((char*)mStateWrapData,	sizeof(mStateWrapData[0])*AGENT_MAX_NUM_STATEVARS);
			// We assume that CAgentActionDescriptor requires nothing special for serialization
			fileOutStream.write((char*)mDescriptors,	sizeof(mDescriptors[0])*AGENT_MAX_NUM_STATEVARS);
			// We don't save mState itself because it will not be used upon revival of an AgentQ
			return true;
		}

		bool	load(std::ifstream &fileInStream)
		{
			fileInStream.read((char*)&mNumStateVars,	sizeof(mNumStateVars));
			fileInStream.read((char*)&mWrapState,		sizeof(mWrapState));
			fileInStream.read((char*)mStateWrapData,	sizeof(mStateWrapData[0])*AGENT_MAX_NUM_STATEVARS);
			fileInStream.read((char*)mDescriptors,		sizeof(mDescriptors[0])*AGENT_MAX_NUM_STATEVARS);
			return true;
		}

		CAgentState& operator=(const CAgentState& state)
		{
			mNumStateVars	= state.mNumStateVars;
			mWrapState		= state.mWrapState;
			memcpy(mState,			state.mState,			mNumStateVars*sizeof(mState[0]));
			memcpy(mStateWrapData,	state.mStateWrapData,	mNumStateVars*sizeof(mState[0]));
			// Copy descriptors
			for (int i=0; i<mNumStateVars; i++)
				mDescriptors[i] = state.mDescriptors[i];

			return *this;
		}

		std::string toStr() const
		{
			std::stringstream ss;
			ss << "[";
			for (int i=0; i<mNumStateVars; i++)
			{
				ss << mState[i];
				if (i < mNumStateVars-1)
					ss << ", ";
			}
			ss << "]";
			return ss.str();
		}
		// Read from xml
		//virtual int		ReadFromXML(XMLFile* xml) = NULL;
};

class CAgentActionDescriptor
{
	public:
		// A total of mNumSteps values will be evaluated, starting at mStartVal, increasing with mStepSize
		_StatePrecision		mStartVal;
		_StatePrecision		mIntervalSize;
		int					mNumSteps;
		_StatePrecision		mStepSize;
		_StatePrecision		mScale;

		void	set(_StatePrecision min, _StatePrecision max, int numSteps, _StatePrecision Scale = 1.0f)
		{
			mNumSteps		= numSteps;
			mStartVal		= min;
			mIntervalSize	= max - min;
			if (numSteps == 1)
				mStepSize	= 0;
			else
				mStepSize	= mIntervalSize / (numSteps - 1.0f);
			mScale	= Scale;
		}

		// Returns the value corresponding to the action index provided
		inline _StatePrecision	getValByIndex(int index)
		{
			if (mNumSteps == 1)
				return mStartVal;
			return mStartVal + index*mIntervalSize/(mNumSteps-1.0f);	// By dividing, the result is more accurate
			//return mStartVal + (mNumSteps == 1) ? (index*mIntervalSize/(mNumSteps-1.0)) : 0;
		}

		// Returns the action index that is closest to the value provided
		inline int				getIndexByVal(_StatePrecision value)
		{
			if (mNumSteps == 1)
				return 0;
			return (int)((value - mStartVal + 0.5*mStepSize)/mStepSize);
		}
};

struct CAgentActionValue
{
	_StatePrecision			mAction[AGENT_MAX_NUM_ACTIONVARS];
	_QPrecision				mValue;			// V or Q value
	_QPrecision				mDistrValue;	// Value of this action as part of a probability distribution
	_QPrecision				mVisits; // Number of times this state/action pair was visited
};

class CAgentActionValueList: public std::vector<CAgentActionValue>
{
	public:
		int					mBestAction;	// Index of the best action in the list

		// getBestAction() assumes that mBestAction is filled with the correct value!
		// It does *not* evaluate all items in the list.
		_StatePrecision*	bestAction()	{return (*this)[mBestAction].mAction;}	// The best action itself
};


class CAgentAction
{
	private:
		int						mNumActionVars;
	protected:
		_StatePrecision			mAction[AGENT_MAX_NUM_ACTIONVARS];

		void init(int numValues)
		{
			mNumActionVars		= numValues;
			memset(mDescriptors, 0, sizeof(mDescriptors[0])*mNumActionVars);
			memset(mAction, 0, sizeof(mAction[0])*mNumActionVars);
		}

	public:
		CAgentActionDescriptor	mDescriptors[AGENT_MAX_NUM_ACTIONVARS];

		CAgentAction(int numValues=1)
		{
			init(numValues);
		}

		CAgentAction(const CAgentAction& actionToCopy)
		{
			init(actionToCopy.mNumActionVars);
			copyDescriptors(actionToCopy);

			// Copy values
			for (int i=0; i<mNumActionVars; i++)
				mAction[i] = actionToCopy.mAction[i];
		}

		inline _StatePrecision*			values()								{return mAction;}
		inline const _StatePrecision*	values() const							{return mAction;}
		inline _StatePrecision&			operator[](unsigned int index)			{return mAction[index];}
		inline const _StatePrecision&	operator[](unsigned int index)	const	{return mAction[index];}

		int				getNumActionVars() const	{ return mNumActionVars; }
		int				getNumActions()	const // Total number of actions; product of all action vars
		{
			int numSteps = 1;
			for (int iActionVar = 0; iActionVar < mNumActionVars; iActionVar++)
				numSteps *= mDescriptors[iActionVar].mNumSteps;
			return numSteps;
		}
		inline _StatePrecision	getActionVar(const int index) const	{return mAction[index];}

		// Get actions scaled by the scaling factor defined in their descriptor
		inline void getScaledAction(_StatePrecision *scaledAction) const
		{
			for (int i=0; i<mNumActionVars; i++)
				scaledAction[i] = mDescriptors[i].mScale*mAction[i];
		}

		// Get actions as integers
		inline void getIntAction(int *intAction) const
		{
			for (int i=0; i<mNumActionVars; i++)
				intAction[i] = (int)(floor(0.5 + mAction[i]/mDescriptors[i].mStepSize));
		}

		// discretize(): returns the action from the allowed action set closest to 'value'
		// You could also call this function 'align'; it aligns the values with the discretized values from the allowed action set
		inline _StatePrecision	discretize(const int index, _StatePrecision value)
		{
			return mDescriptors[index].getValByIndex(mDescriptors[index].getIndexByVal(value));
		}

		inline void		discretize()
		{
			for (int i=0; i<mNumActionVars; i++)
				mAction[i] = discretize(i, mAction[i]);
		}

		inline void		setStartValues()
		{
			for (int i=0; i<mNumActionVars; i++)
			{
				mAction[i] = mDescriptors[i].mStartVal;


			}

		}
		inline void		setStartZeroValues()
				{
					for (int i=0; i<mNumActionVars; i++)
					{
						//reset at zero
						mAction[i] = 0;
					}

				}

		// store(): fills the provided parameter with values from mAction
		inline void		store(_StatePrecision *valuesOut) const
		{
			memcpy(valuesOut, mAction, mNumActionVars*sizeof(_StatePrecision));
		}

		// restore(): fills mAction with the values provided
		inline void		restore(_StatePrecision *valuesIn)
		{
			memcpy(mAction, valuesIn, mNumActionVars*sizeof(_StatePrecision));
		}
/*
		void			randomizeUniform(TRanrotBGenerator* rg)
		{
			for (int i=0; i<mNumActionVars; i++)
				mAction[i] = mDescriptors[i].getValByIndex(rg->IRandom(0, mDescriptors[i].mNumSteps-1));  // Only choose values from the actual discrete action values, not in between.
		}

		int				randomizeGibbs(CAgentActionValueList* avList, _QPrecision temperature, TRanrotBGenerator* rg)
		{
			int numActions = getNumActions();
			_QPrecision gibbsSum = 0;
			for (int i=0; i<numActions; i++)
			{
				(*avList)[i].mDistrValue = exp((*avList)[i].mValue/temperature);
				gibbsSum += (*avList)[i].mDistrValue;
			}
			double normFact		= 1/gibbsSum;	// Normalization factor
			double randomNumber = rg->Random();	// Random number
			double cumulProb	= 0;			// Cumulative probability sum
			for (int i=0; i<numActions; i++)
			{
				cumulProb += (*avList)[i].mDistrValue*normFact;
				if (randomNumber <= cumulProb)
				{
					// We arrived at our random choice!
					restore((*avList)[i].mAction);
					return i;
				}
			}

			// We should never get here! Return best action.
			printf("Error: cumulProb=%.16g, normFact=%.16g. Returning best action.\n", cumulProb, normFact);
			restore((*avList)[avList->mBestAction].mAction);
			return avList->mBestAction;
		}
*/
		CAgentAction& operator=(const CAgentAction& action)
		{
			// Reinit if numvalues is not equal.
			// This is done in CopyDescriptors, which can be called separately

			// Copy descriptors
			copyDescriptors(action);

			// Copy values
			for (int i=0; i<mNumActionVars; i++)
				mAction[i] = action.mAction[i];

			return *this;
		}

		void			copyDescriptors(const CAgentAction& actionToCopy)
		{
			// Reinit if numvalues is not equal
			if (actionToCopy.mNumActionVars != mNumActionVars)
				init(actionToCopy.mNumActionVars);

			// Copy descriptors
			for (int i=0; i<mNumActionVars; i++)
				mDescriptors[i] = actionToCopy.mDescriptors[i];
		}

		void			setNumActionVars(int numValues)
		{
			init(numValues);
		}

		bool	save(std::ofstream &fileOutStream)
		{
			fileOutStream.write((char*)&mNumActionVars,	sizeof(mNumActionVars));
			// We assume that CAgentActionDescriptor requires nothing special for serialization
			fileOutStream.write((char*)mDescriptors,	sizeof(mDescriptors[0])*AGENT_MAX_NUM_ACTIONVARS);
			// We don't save mAction itself because it will not be used upon revival of an AgentQ
			return true;
		}

		bool	load(std::ifstream &fileInStream)
		{
			fileInStream.read((char*)&mNumActionVars,	sizeof(mNumActionVars));
			fileInStream.read((char*)mDescriptors,		sizeof(mDescriptors[0])*AGENT_MAX_NUM_ACTIONVARS);
			return true;
		}

		std::string toStr() const
		{
			std::stringstream ss;
			ss << "[";
			for (int i=0; i<mNumActionVars; i++)
			{
				ss << mAction[i];
				if (i < mNumActionVars-1)
					ss << ", ";
			}
			ss << "]";
			return ss.str();
		}
};


#endif /* AGENTINTERFACE_H_ */
