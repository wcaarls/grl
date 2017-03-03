/*
 * Statistics.cpp
 *
 *  Created on: Dec 3, 2008
 *      Author: Erik Schuitema
 */

#include <grl/statistics.h>
#include <math.h>
#include <float.h>
#include <algorithm>
#include <string.h>

CSimpleStat::CSimpleStat()
{
  mSampleBuffer  = NULL;
  mBufLength    = 0;
  clear();
}

CSimpleStat::CSimpleStat(const int bufferlength)
{
  mSampleBuffer  = NULL;
  mBufLength    = 0;
  setBufferLength(bufferlength);
}

CSimpleStat::~CSimpleStat()
{
  if (mSampleBuffer != NULL)
    delete[] mSampleBuffer;
}

void CSimpleStat::setBufferLength(const int length)
{
  if (mSampleBuffer != NULL)
    delete[] mSampleBuffer;
  mSampleBuffer = new double[length];
  mBufLength = length;
  clear();
}

void CSimpleStat::clear()
{
  if (mSampleBuffer != NULL)
    memset(mSampleBuffer, 0, mBufLength*sizeof(double));
  mBufPos   = 0;
  mAvgSum    = 0;
  mSqrSum    = 0;
  mAvgSumShd  = 0;
  mSqrSumShd  = 0;
  mHasWrapped  = false;
  resetMinMax();
}

void CSimpleStat::addValue(double value)
{
  // Remove old sample from sums (initialized as zero)
  mAvgSum    -= mSampleBuffer[mBufPos];
  mSqrSum    -= mSampleBuffer[mBufPos]*mSampleBuffer[mBufPos];

  // Update sample buffer
  mSampleBuffer[mBufPos] = value;

  // Add new sample to sums. Also update shadow sums.
  mAvgSum    += value;
  mAvgSumShd  += value;
  double sqrNewVal = value*value;
  mSqrSum    += sqrNewVal;
  mSqrSumShd  += sqrNewVal;

  // Update minimum and maximum
  if (value < mMin)
    mMin = value;
  if (value > mMax)
    mMax = value;

  // Increment position in circular buffer
  mBufPos++;
  if (mBufPos >= mBufLength)
  {
    mBufPos    = 0;
    mHasWrapped  = true;

    // Swap sums
    mAvgSum = mAvgSumShd;
    mSqrSum = mSqrSumShd;
    // Reset shadow sums
    mAvgSumShd = 0;
    mSqrSumShd = 0;
  }
}

double CSimpleStat::getAverage() const
{
  if (mHasWrapped)
    return mAvgSum/(double)mBufLength;
  else
    return mAvgSum/(double)std::max(mBufPos,1);
}

double CSimpleStat::getVariance() const
{
  // Biased estimator (we divide by N instead of N-1)
  double oneOverN;
  if (mHasWrapped)
    oneOverN = 1.0/(double)mBufLength;
  else // In this case, mBufPos equals number of samples
    oneOverN = 1.0/(double)std::max(mBufPos,1);

  double avgSumOverN = mAvgSum*oneOverN;
  return (mSqrSum*oneOverN) - (avgSumOverN*avgSumOverN);
}

double CSimpleStat::getStdev() const
{
  return sqrt(std::max(0.0, getVariance()));
}

double CSimpleStat::getMinimum() const
{
  return mMin;
}

double CSimpleStat::getMaximum() const
{
  return mMax;
}

void CSimpleStat::resetMinMax()
{
  mMin     =  DBL_MAX;
  mMax     = -DBL_MAX;  // NOT DBL_MIN since this is a POSITIVE number very close to zero!!!!
}

double CSimpleStat::getBufVal(int index)
{
  return mSampleBuffer[index];
}

int CSimpleStat::getNumValues() const
{
  if (mHasWrapped)
    return mBufLength;
  else
    return mBufPos;
}

int CSimpleStat::getLength() const
{
  return mBufLength;
}

void CSimpleStat::getHistogram(int *bins, int numBins, double minVal, double maxVal) const
{
  int numSamples;
  if (mHasWrapped)
    numSamples = mBufLength;
  else
    numSamples = mBufPos;

  // Reset bins
  for (int iBin=0; iBin<numBins; iBin++)
    bins[iBin] = 0;

  double binSize = (maxVal - minVal)/(double)numBins;
  for (int iSample=0; iSample<numSamples; iSample++)
  {
    double binIndex = floor((mSampleBuffer[iSample] - minVal)/binSize);
    binIndex = std::min(std::max(binIndex, 0.0), numBins-1.0);
    bins[(int)binIndex]++;
  }
}

std::string CSimpleStat::toStr(const std::string& unitString) const
{
  std::stringstream ss;
  if ((mBufPos <= 0) && !mHasWrapped)
    ss << "No data";
  else
  {
    double avg = getAverage();
    double stdev = getStdev();
    ss.setf(std::ios::fixed,std::ios::floatfield);
    ss.precision(2);
    ss << "Avg: " << avg << unitString << "\tStdev: " << stdev << unitString << " (" << 100.0*stdev/avg << "%)\tMin: " << getMinimum() << unitString << "\tMax:" << getMaximum() << unitString;
  }
  return ss.str();
}
