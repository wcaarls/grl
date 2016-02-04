/*
 * MedianFilter.h
 *
 *  Created on: Dec 11, 2008
 *      Author: Erik Schuitema
 */

#ifndef MEDIANFILTER_H_
#define MEDIANFILTER_H_

#include "Filter.h"
//#include <stdio.h>
//#include <algorithm>

// Median filter working with sliding window
// In order to get a real median, use an odd number for FILTERLENGTH
template<class FILTERTYPE, int FILTERLENGTH>
class CMedianFilter: public CFilter<FILTERTYPE, FILTERLENGTH>
{
	protected:
		// Sample buffer is sorted automatically whenever a new sample is added
		FILTERTYPE	mSortedBuffer[FILTERLENGTH];
		int			mBufPos;
		bool		mHasWrapped;	// Indicates whether we filled the buffer completely at least once yet
		void		resortNewSample(const int sampleIndex);
		int			getSortedSampleIndex(const FILTERTYPE sampleValue);
		void		moveSortedSample(const int sampleIndex, const int numPlaces);
	public:
		CMedianFilter();

		//void		AddValue(FILTERTYPE value);
		FILTERTYPE	filter(FILTERTYPE newSample);
		FILTERTYPE	getFilteredValue();
		void		clear();
};

// ************************** CMedianFilter **************************//
template<class FILTERTYPE, int FILTERLENGTH>
CMedianFilter<FILTERTYPE, FILTERLENGTH>::CMedianFilter()
{
	mBufPos 	= 0;
	mHasWrapped	= false;
	memset(mSortedBuffer, 0, FILTERLENGTH*sizeof(FILTERTYPE));
}

template<class FILTERTYPE, int FILTERLENGTH>
void CMedianFilter<FILTERTYPE, FILTERLENGTH>::clear()
{
	CFilter<FILTERTYPE, FILTERLENGTH>::clear();
	mBufPos		= 0;
	mHasWrapped	= false;
	memset(mSortedBuffer, 0, FILTERLENGTH*sizeof(FILTERTYPE));
}

template<class FILTERTYPE, int FILTERLENGTH>
FILTERTYPE CMedianFilter<FILTERTYPE, FILTERLENGTH>::filter(FILTERTYPE newSample)
{
	// Put new sample in sorted buffer
	int sortedSampleIndex;
	if (mHasWrapped)
		// Replace existing sample
		sortedSampleIndex = getSortedSampleIndex(this->mSampleBuffer[mBufPos]);	// Request position of old sample
	else
		// Just add new sample at the end of the sorted buffer
		sortedSampleIndex = mBufPos;

	mSortedBuffer[sortedSampleIndex] = newSample;
	// Put new sample in sample buffer
	this->mSampleBuffer[mBufPos] = newSample;
	// Keep track of our position and whether we have wrapped
	mBufPos++;
	if (mBufPos >= FILTERLENGTH)
	{
		mBufPos		= 0;
		mHasWrapped	= true;
	}
	// Resort the buffer
	resortNewSample(sortedSampleIndex);
	// Return the median
	return getFilteredValue();
}

template<class FILTERTYPE, int FILTERLENGTH>
FILTERTYPE CMedianFilter<FILTERTYPE, FILTERLENGTH>::getFilteredValue()
{
	// Return the middle value: the median
	if (mHasWrapped)
		return mSortedBuffer[FILTERLENGTH/2];
	else
		return mSortedBuffer[mBufPos/2];
}


template<class FILTERTYPE, int FILTERLENGTH>
int CMedianFilter<FILTERTYPE, FILTERLENGTH>::getSortedSampleIndex(const FILTERTYPE sampleValue)
{
	int bufSize = mHasWrapped?FILTERLENGTH:(mBufPos+1);
	int index;
	for (index=0; index<bufSize; index++)
		if (mSortedBuffer[index] == sampleValue)
			break;

	return index;
}

template<class FILTERTYPE, int FILTERLENGTH>
void CMedianFilter<FILTERTYPE, FILTERLENGTH>::moveSortedSample(const int sampleIndex, const int numPlaces)
{
	FILTERTYPE ourSample = mSortedBuffer[sampleIndex];
	if (numPlaces > 0)
	{
		// This means that data needs to move down
		for (int i=sampleIndex; i<sampleIndex+numPlaces; i++)
			mSortedBuffer[i] = mSortedBuffer[i+1];
		mSortedBuffer[sampleIndex+numPlaces] = ourSample;
	}

	if (numPlaces < 0)
	{
		// This means that data needs to move up
		for (int i=sampleIndex; i>sampleIndex+numPlaces; i--)
			mSortedBuffer[i] = mSortedBuffer[i-1];
		mSortedBuffer[sampleIndex+numPlaces] = ourSample;
	}
}

// Move a single sample so that the already sorted list remains sorted
template<class FILTERTYPE, int FILTERLENGTH>
void CMedianFilter<FILTERTYPE, FILTERLENGTH>::resortNewSample(const int sampleIndex)
{
	int bufSize = mHasWrapped?FILTERLENGTH:(mBufPos);

	// First, look up
	int searchIndex = sampleIndex;
	while (searchIndex+1 < bufSize)
	{
		if (mSortedBuffer[searchIndex+1] < mSortedBuffer[sampleIndex])
			searchIndex++;
		else
			break;
	}

	// Then, look down
	//searchIndex = sampleIndex;
	while(searchIndex-1 >= 0)
	{
		if (mSortedBuffer[searchIndex-1] > mSortedBuffer[sampleIndex])
			searchIndex--;
		else
			break;
	}
	moveSortedSample(sampleIndex, searchIndex-sampleIndex);

	/*
	printf(" Sort done: [ ");
	for (int i=0; i<bufSize; i++)
		printf("%d ", mSortedBuffer[i]);
	printf("] ");
	*/
}

#endif /* MEDIANFILTER_H_ */
