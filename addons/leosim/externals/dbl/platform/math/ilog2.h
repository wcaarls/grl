/*
 * ilog2.h
 *
 * Fast integer log2 functions, only valid for x86 architectures using the BSR instruction
 *
 *  Created on: Sep 9, 2010
 *      Author: Erik Schuitema
 */

#ifndef ILOG2_H_
#define ILOG2_H_

#ifdef _MSC_VER
#ifdef _WIN64
static inline int64_t ilog2_x86(int64_t x)
{
	unsigned long retval;
	_BitScanReverse64(&retval, x);
	return retval;
}
#else
static inline int16_t ilog2_x86(int16_t x)
{
	int16_t retval;
	__asm {
		bsr ax, x
		mov retval, ax
	}
	return retval;
}
#endif
static inline int32_t ilog2_x86(int32_t x)
{
	unsigned long retval;
	_BitScanReverse(&retval, x);
	return retval;
}
#else
static inline int16_t ilog2_x86(int16_t x)
{
	int16_t retval;
	asm ("bsr %1, %0" : "=r" (retval) : "r" (x));
	return retval;
}
static inline int32_t ilog2_x86(int32_t x)
{
	int32_t retval;
	asm ("bsr %1, %0" : "=r" (retval) : "r" (x));
	return retval;
}
static inline int64_t ilog2_x86(int64_t x)
{
	int64_t retval;
	asm ("bsr %1, %0" : "=r" (retval) : "r" (x));
	return retval;
}
#endif

#endif /* ILOG2_H_ */
