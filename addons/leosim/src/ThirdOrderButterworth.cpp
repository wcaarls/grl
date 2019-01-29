/*
 * ThirdOrderButterworth.cpp
 *
 *  Created on: Nov 21, 2008
 *      Author: Erik Schuitema
 */

#include <grl/environments/leosim/ThirdOrderButterworth.h>

CThirdOrderButterworthFilter::CThirdOrderButterworthFilter()
{
	memset(mKin, 0, 4*sizeof(double));
	memset(mKout, 0, 4*sizeof(double));

	// Make it a pass-through filter by default
	mKin[0] = 1.0;
}

void CThirdOrderButterworthFilter::init(double samplingFrequency, double cutoffFrequency)
{
	/*
	 * The transfer function of a continuous-time third order Butterworth filter is as follows:
	 *
	 *  H(s) = 1 / ( (s/wc)^3 + 2(s/wc)^2 + 2(s/wc) + 1 )
	 *
	 * where wc (omega_c) is the cutoff frequency in radians (w = 2*pi*f)
	 *
	 * Digitizing this transfer function to become H(z) using Tustin's method involves the replacement:
	 *
	 *  s = ( 2 / T ) * ( ( z - 1 ) / ( z + 1) )
	 *
	 * where T is the sample period.
	 * Expanding this (by hand, muahaha) results in the discrete-time transfer function:
	 *
	 *  H(z) = ( mKin[0]  + mKin[1]  * z^-1 + mKin[2]  * z^-2 + mKin[3]  * z^-3 ) /
	 *         ( mKout[0] + mKout[1] * z^-1 + mKout[3] * z^-2 + mKout[3] * z^-3 )
	 *
	 * with the coefficients as given below.
	 *
	 * Later, I found the following URL, which also shows derived formulas:
	 * http://www.planetanalog.com/showArticle.jhtml?articleID=12802683&pgno=3
	 *
	 * - Erik Schuitema
	 */

	double T = 2.0*M_PI*cutoffFrequency/samplingFrequency;
	double normalizeOutput = T*T*T + 4.0*T*T + 8.0*T + 8.0;
	mKout[1]	= (3.0*T*T*T + 4.0*T*T - 8.0*T - 24.0)/normalizeOutput;
	mKout[2]	= (3.0*T*T*T - 4.0*T*T - 8.0*T + 24.0)/normalizeOutput;
	mKout[3]	= (    T*T*T - 4.0*T*T + 8.0*T -  8.0)/normalizeOutput;

	mKin[0]		=   T*T*T/normalizeOutput;
	mKin[1]		= 3*T*T*T/normalizeOutput;
	mKin[2]		= 3*T*T*T/normalizeOutput;
	mKin[3]		=   T*T*T/normalizeOutput;

	// Print filter coefficients for debugging
	//printf("b0 = %.5f\tb1 = %.5f\tb2 = %.5f\tb3 = %.5f\n", mKin[0], mKin[1], mKin[2], mKin[3]);
	//printf("a0 = %.5f\ta1 = %.5f\ta2 = %.5f\ta3 = %.5f\n", mKout[0], mKout[1], mKout[2], mKout[3]);

	memset(mFilterBuffer, 0, 4*sizeof(double));
}

double CThirdOrderButterworthFilter::filter(double newSample)
{
	mSampleBuffer[3] = mSampleBuffer[2];
	mSampleBuffer[2] = mSampleBuffer[1];
	mSampleBuffer[1] = mSampleBuffer[0];
	mSampleBuffer[0] = newSample;

	mFilterBuffer[3] = mFilterBuffer[2];
	mFilterBuffer[2] = mFilterBuffer[1];
	mFilterBuffer[1] = mFilterBuffer[0];

	// The actual filtering step
	mFilterBuffer[0] = mKin[0]*mSampleBuffer[0] + mKin[1] *mSampleBuffer[1] + mKin[2] *mSampleBuffer[2] + mKin[3] *mSampleBuffer[3] +
						- mKout[1]*mFilterBuffer[1] - mKout[2]*mFilterBuffer[2] - mKout[3]*mFilterBuffer[3];

	return mFilterBuffer[0];
}

double CThirdOrderButterworthFilter::getValue()
{
	return mFilterBuffer[0];
}

