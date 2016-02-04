/*
 * Statistics.hpp
 *
 *  Created on: Nov 26, 2008
 *      Author: Erik Schuitema
 */

#ifndef STATISTICS_HPP_
#define STATISTICS_HPP_

#include <sstream>

// Simple statistics class to calculate minimum, maximum, average, variance and standard deviation of doubles
class CSimpleStat
{
	protected:
		// Window filtering
		double		*mSampleBuffer;
		int			mBufLength;
		int			mBufPos;
		double		mAvgSum;
		double		mSqrSum;
		double		mMin;
		double		mMax;
		bool		mHasWrapped;	// Indicates whether we filled the buffer completely at least once yet

		// To avoid the build-up of numerical errors, every 'cycle' (wrap) of the buffer,
		// we keep shadow sums of every new cycle of the buffer.
		// When the buffer wraps, the current sums are replaced by the fresh shadow sums
		double		mAvgSumShd;
		double		mSqrSumShd;

	public:
		CSimpleStat();
		CSimpleStat(const int bufferlength);
		~CSimpleStat();
		void		setBufferLength(const int length);
		void		clear();
		void		addValue(double value);
		double		getVariance() const;
		double		getAverage() const;
		double		getStdev() const;
		double		getMinimum() const;
		double		getMaximum() const;
		void		getHistogram(int *bins, int numBins, double minVal, double maxVal) const;

		double		getBufVal(int index);	// Mainly for debugging purposes
		int			getLength() const;
		int			getNumValues() const;	// Returns the number of added values. This cannot be larger than mBufLength.
		std::string	toStr(const std::string& unitString) const;	// You can pass the units (e.g. us, ms, kg) as string

		void		resetMinMax();
};


#endif /* STATISTICS_HPP_ */
