#ifndef PRECISIONS_H_
#define PRECISIONS_H_

#include <half.h>
#include <stdint.h>
#include <float.h>

#define _StatePrecision	float
#define _StatePrecision_MAX FLT_MAX
#ifdef QPRECISIONDOUBLE
	#define _QPrecision double
	#define _QPrecision_MAX DBL_MAX
//#warning Using double precision floating point!
#else
	#define _QPrecision float
	#define _QPrecision_MAX FLT_MAX
#endif
//#define _QPrecision half
//#define _QPrecision_MAX half::posInf()
#define _IndexPrecision uint32_t		// Should be unsigned
#define _IndexPrecision_MAX INT_MAX

#endif /* PRECISIONS_H_ */
