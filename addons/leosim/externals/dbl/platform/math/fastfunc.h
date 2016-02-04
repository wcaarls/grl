/*
 * fastfunc.h
 *
 * Fast approximation of an arbitrary function using a lookup-table.
 * Support for linear interpolation and nearest neighbor queries.
 *
 *  Created on: Jan 13, 2011
 *      Author: Erik Schuitema
 */

#ifndef FASTFUNC_H_
#define FASTFUNC_H_

#include <assert.h>
#include <cmath>

#define _fastfunc_datatype float

class CFastFunc
{
	protected:
		_fastfunc_datatype	_min;
		_fastfunc_datatype	_step;
		_fastfunc_datatype	_one_over_step;
		int					_length;
		_fastfunc_datatype*	_lut;
	public:
		// Some functions for your convenience that can be passed to init()
		static _fastfunc_datatype gauss(_fastfunc_datatype x)	{return std::exp(-x*x);}
		static _fastfunc_datatype gauss2(_fastfunc_datatype xx)	{return std::exp(-xx);}	// Use when you already squared the argument
		static _fastfunc_datatype exp(_fastfunc_datatype x)		{return std::exp(x);}

		CFastFunc():
			_min(0),
			_step(0),
			_one_over_step(0),
			_length(0),
			_lut(NULL)
		{}
		~CFastFunc()
		{
			if (_lut != NULL)
				delete[] _lut;
		}

		// You may call fastfunc_init() repeatedly.
		void init(_fastfunc_datatype min, _fastfunc_datatype max, int numpoints, _fastfunc_datatype (*function)(_fastfunc_datatype))
		{
			if (_lut != NULL)
				delete[] _lut;

			_min			= min;
			_length			= numpoints;
			_step			= (max-min)/((_fastfunc_datatype)(numpoints-1));
			_one_over_step	= ((_fastfunc_datatype)(numpoints-1))/(max-min);
			_lut			= new _fastfunc_datatype[numpoints];
			for (int i=0; i<numpoints; i++)
				_lut[i] = (*function)(min + i*_step);
		}

		// Get function value using linear interpolation.
		// When x is outside the init region, zero order hold is applied
		inline _fastfunc_datatype getValue(_fastfunc_datatype x)
		{
			assert(_lut != NULL);

			// Calculate first index.
			int index1 = (int)((x - _min)*_one_over_step);
			// Use zero order hold outside the precalculated range
			if (x < _min)
				return _lut[0];
			int index2 = index1+1;
			if (index2 > _length-1)
				return _lut[_length-1];
			// Return linear interpolation
			return _lut[index1] + ((x - _min - index1*_step)*_one_over_step)*(_lut[index2] - _lut[index1]);
		}

		// Get function value using linear interpolation.
		// 'x' must be inside the init region.
		// When x is outside the init region, this function fails!
		// This function is not so robust against floating point roundoff errors in the calculation, so be careful!
		inline _fastfunc_datatype getValueInRange(_fastfunc_datatype x)
		{
			assert(_lut != NULL);

			// Calculate first and second index
			int index1 = (int)((x - _min)*_one_over_step);
			int index2 = index1+1;
			assert(index1 >= 0);
			assert(index2 < _length);
			// Return linear interpolation
			return _lut[index1] + ((x - _min - index1*_step)*_one_over_step)*(_lut[index2] - _lut[index1]);
		}

		// Get function value using the nearest neighbor
		// When x is outside the init region, zero order hold is applied
		inline _fastfunc_datatype getNN(_fastfunc_datatype x)
		{
			assert(_lut != NULL);

			// Calculate nearest neighbor index.
			int index = (int)(0.5 + (x - _min)*_one_over_step);
			// Use zero order hold outside the precalculated range
			if (index < 0)
				return _lut[0];
			if (index > _length-1)
				return _lut[_length-1];
			// Return NN
			return _lut[index];
		}

		// Get function value using the nearest neighbor.
		// 'x' must be inside the init region.
		// When x is outside the init region, this function fails!
		// Since we use rounding, this function is quite robust against floating point roundoff errors in the calculation.
		inline _fastfunc_datatype getNNInRange(_fastfunc_datatype x)
		{
			assert(_lut != NULL);

			// Calculate nearest neighbor index.
			int index = (int)(0.5 + (x - _min)*_one_over_step);
			assert(index >= 0);
			assert(index < _length);
			// Return NN
			return _lut[index];
		}
};

// Test function
inline void fastfunc_test()
{
	//FILE* fOut = stdout;
	FILE* fOut = fopen("fastfunctest.txt", "wt");
	_fastfunc_datatype min = -2;
	_fastfunc_datatype max = 0;
	int numpoints = 20;
	int numsamples = 100;
	CFastFunc fastfunc;
	fastfunc.init(min, max, numpoints, &CFastFunc::gauss);
	for (int i=0; i<numsamples; i++)
	{
		_fastfunc_datatype x = min + i*(max-min)/(numsamples-1);
		fprintf(fOut, "%.8f\t%.8f\n", x, fastfunc.getValue(x));
	}
	fclose(fOut);
}

#endif /* FASTFUNC_H_ */
