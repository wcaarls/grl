/*
 * MovingAverage.h
 *
 *  Created on: Nov 24, 2008
 *      Author: Erik Schuitema
 */

#ifndef MOVINGAVERAGE_H_
#define MOVINGAVERAGE_H_

#include "Filter.h"
#include <algorithm>

// Simple moving average class (as opposed to weighted moving average, etc.)
template<class FILTERTYPE, int FILTERLENGTH>
class CSimpleMovingAverage:	public CFilter<FILTERTYPE, FILTERLENGTH>
{
	protected:
		// Moving average is calculated incrementally
		//double		mSampleBuffer[FILTERLENGTH];
		int			mBufPos;
		FILTERTYPE	mSum;
		bool		mHasWrapped;	// Indicates whether we filled the buffer completely at least once yet
	public:
		CSimpleMovingAverage();

		//void		AddValue(FILTERTYPE value);
		FILTERTYPE	filter(FILTERTYPE newSample);
		FILTERTYPE	getFilteredValue();
};

// **************** CSimpleMovingAverage ***************** //
template<class FILTERTYPE, int FILTERLENGTH>
CSimpleMovingAverage<FILTERTYPE, FILTERLENGTH>::CSimpleMovingAverage()
{
	mBufPos 	= 0;
	mSum		= 0;
	mHasWrapped	= false;
}

template<class FILTERTYPE, int FILTERLENGTH>
FILTERTYPE CSimpleMovingAverage<FILTERTYPE, FILTERLENGTH>::filter(FILTERTYPE newSample)
{
	mSum -= this->mSampleBuffer[mBufPos];	// Because "base template classes are not in the default lookup", use the workaround "this->member"
	this->mSampleBuffer[mBufPos] = newSample;
	mSum += newSample;
	mBufPos++;
	if (mBufPos >= FILTERLENGTH)
	{
		mBufPos		= 0;
		mHasWrapped	= true;
	}

	return getFilteredValue();
}

template<class FILTERTYPE, int FILTERLENGTH>
FILTERTYPE CSimpleMovingAverage<FILTERTYPE, FILTERLENGTH>::getFilteredValue()
{
	if (mHasWrapped)
		return mSum/(double)FILTERLENGTH;
	else
		return mSum/(double)std::max(mBufPos, 1);
}

#endif /* MOVINGAVERAGE_H_ */
