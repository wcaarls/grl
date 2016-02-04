/*
 * ThirdOrderButterworth.h
 *
 *  Created on: Nov 21, 2008
 *      Author: Erik Schuitema
 */

#ifndef THIRDORDERBUTTERWORTH_H_
#define THIRDORDERBUTTERWORTH_H_

#include "Filter.h"
#include <math.h>
#include <string.h>

//#ifndef M_PI
//  #define M_PI 3.14159265358979323846
//#endif

// ****************** NOT FINISHED YET ****************** //
template<int ORDER>
class CButterworthFilter: public CFilter<double, ORDER+1>
{
	protected:
		// Filter constants
		double	mKout[ORDER+1];	//mKout[0] is not used; it is the output
		double	mKin[ORDER+1];
		// Filter buffer
		double	mFilterBuffer[ORDER+1];	// Sample[0] is the most recent; Sample[1] is 1 update old, etc.
		void	shiftFilterBuffer()
		{
			for (int i = ORDER; i>0; i--)	// Now let's hope that the compiler unrolls this loop
				mFilterBuffer[i] = mFilterBuffer[i-1];
		}
	public:
		CButterworthFilter()
		{
			// Make it a pass-through filter by default
			memset(mKout, 0, (ORDER+1)*sizeof(double));
			memset(mKin, 0, (ORDER+1)*sizeof(double));
			mKin[0] = mKout[0] = 1.0;
		}
		void	init(double samplingFrequency, double cutoffFrequency)
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
			 *         ( mKout[0] + mKout[1] * z^-1 + mKout[2] * z^-2 + mKout[3] * z^-3 )
			 *
			 * with the coefficients as given below and where mKout[0] == 1.
			 *
			 * Later, I found the following URL, which also shows derived formulas:
			 * http://www.planetanalog.com/showArticle.jhtml?articleID=12802683&pgno=3
			 *
			 * The same was done for the first and second order filters
			 *
			 * - Erik Schuitema
			 */

			// Take the cutoff frequency into account by using s/w_c instead of s.
			// When using Tustin, this becomes s/w_c = (2/(T*w_c))*((z-1)/(z+1)).
			// Therefore, whenever T appears in the formulas, we use T*w_c = T*2*pi*f_c = (1/f_s)*2*pi*f_c.
			double T = 2.0*M_PI*cutoffFrequency/samplingFrequency;

			switch(ORDER)
			{
				case 1:
					{
						double normalizeOutput	= T + 2.0;
						mKout[1]	= (T - 2.0)/normalizeOutput;

						mKin[0]		= T/normalizeOutput;
						mKin[1]		= T/normalizeOutput;
					}
					break;
				case 2:
					{
						double normalizeOutput	= T*T + 2.0*sqrt(2.0)*T + 4.0;
						mKout[1]	= (2.0*T*T                   - 8.0)/normalizeOutput;
						mKout[2]	= (    T*T - 2.0*sqrt(2.0)*T + 4.0)/normalizeOutput;

						mKin[0]		=     T*T/normalizeOutput;
						mKin[1]		= 2.0*T*T/normalizeOutput;
						mKin[2]		=     T*T/normalizeOutput;
					}
					break;
				case 3:
					{
						double normalizeOutput = T*T*T + 4.0*T*T + 8.0*T + 8.0;
						mKout[1]	= (3.0*T*T*T + 4.0*T*T - 8.0*T - 24.0)/normalizeOutput;
						mKout[2]	= (3.0*T*T*T - 4.0*T*T - 8.0*T + 24.0)/normalizeOutput;
						mKout[3]	= (    T*T*T - 4.0*T*T + 8.0*T -  8.0)/normalizeOutput;

						mKin[0]		=   T*T*T/normalizeOutput;
						mKin[1]		= 3*T*T*T/normalizeOutput;
						mKin[2]		= 3*T*T*T/normalizeOutput;
						mKin[3]		=   T*T*T/normalizeOutput;

					}
					break;
			}
			// Print filter coefficients for debugging
			/*
			for (int iIn=0; iIn<ORDER+1; iIn++)
				printf("b%d = %.5f\t", mKin[iIn]);
			printf("\n");
			for (int iOut=0; iOut<ORDER+1; iOut++)
				printf("a%d = %.5f\t", mKout[iOut]);
			printf("\n");
			*/

			memset(mFilterBuffer, 0, (ORDER+1)*sizeof(double));
		}

		double	filter(double newSample)
		{
			this->addSample(newSample);
			shiftFilterBuffer();

			// The actual filtering step
			mFilterBuffer[0] = 0;
			for (int iIn=0; iIn<=ORDER; iIn++)
				mFilterBuffer[0] += mKin[iIn]*this->mSampleBuffer[iIn];
			for (int iOut=1; iOut<=ORDER; iOut++)
				mFilterBuffer[0] -= mKout[iOut]*mFilterBuffer[iOut];

			return mFilterBuffer[0];
		}
		void	clear()
		{
			CFilter<double, ORDER+1>::clear();
			memset(mFilterBuffer, 0, (ORDER+1)*sizeof(mFilterBuffer[0]));
		}

		double	getValue()	{return mFilterBuffer[0];}	// Returns the most recent value
};

class CThirdOrderButterworthFilter: public CFilter<double, 4>
{
	protected:
		// Filter constants
		double	mKout[4];	//mKout[0] is not used; it is the output
		double	mKin[4];

		// Sample[0] is the most recent; Sample[3] is 3 updates old
		double	mFilterBuffer[4];
	public:
		CThirdOrderButterworthFilter();
		void	init(double samplingFrequency, double cutoffFrequency);
		double	filter(double newSample);
		double	getValue();	// Returns the most recent value
};


#endif /* THIRDORDERBUTTERWORTH_H_ */
