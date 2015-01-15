//----------------------------------------------------------------------
// File:			pr_queue_k.h
// Programmer:		Sunil Arya and David Mount
// Description:		Include file for priority queue with k items.
// Last modified:	01/04/05 (Version 1.0)
//----------------------------------------------------------------------
// Copyright (c) 1997-2005 University of Maryland and Sunil Arya and
// David Mount.  All Rights Reserved.
// 
// This software and related documentation is part of the Approximate
// Nearest Neighbor Library (ANN).  This software is provided under
// the provisions of the Lesser GNU Public License (LGPL).  See the
// file ../ReadMe.txt for further information.
// 
// The University of Maryland (U.M.) and the authors make no
// representations about the suitability or fitness of this software for
// any purpose.  It is provided "as is" without express or implied
// warranty.
//----------------------------------------------------------------------
// History:
//	Revision 0.1  03/04/98
//		Initial release
//----------------------------------------------------------------------

#ifndef PR_QUEUE_K_H
#define PR_QUEUE_K_H

#include <ANN/ANNx.h>					// all ANN includes
#include <ANN/ANNperf.h>				// performance evaluation

//----------------------------------------------------------------------
//	Basic types
//----------------------------------------------------------------------
typedef ANNdist			PQKkey;			// key field is distance
typedef int				PQKinfo;		// info field is int

//----------------------------------------------------------------------
//	Constants
//		The NULL key value is used to initialize the priority queue, and
//		so it should be larger than any valid distance, so that it will
//		be replaced as legal distance values are inserted.  The NULL
//		info value must be a nonvalid array index, we use ANN_NULL_IDX,
//		which is guaranteed to be negative.
//----------------------------------------------------------------------

const PQKkey	PQ_NULL_KEY  =  ANN_DIST_INF;	// nonexistent key value
const PQKinfo	PQ_NULL_INFO =  ANN_NULL_IDX;	// nonexistent info value
#ifdef ANN_REALTIME
	#ifndef ANN_MAX_K
		#define ANN_MAX_K 100
	#endif
#endif

//----------------------------------------------------------------------
//	ANNmin_k
//		An ANNmin_k structure is one which maintains the smallest
//		k values (of type PQKkey) and associated information (of type
//		PQKinfo).  The special info and key values PQ_NULL_INFO and
//		PQ_NULL_KEY means that thise entry is empty.
//
//		It is currently implemented using an array with k items.
//		Items are stored in increasing sorted order, and insertions
//		are made through standard insertion sort.  (This is quite
//		inefficient, but current applications call for small values
//		of k and relatively few insertions.)
//		
//		Note that the list contains k+1 entries, but the last entry
//		is used as a simple placeholder and is otherwise ignored.
//----------------------------------------------------------------------

class ANNmin_k {
	struct mk_node {					// node in min_k structure
		PQKkey			key;			// key value
		PQKinfo			info;			// info field (user defined)
	};

	int			k;						// max number of keys to store
	int			n;						// number of keys currently active
#ifdef ANN_REALTIME
	mk_node		mk[ANN_MAX_K];
#else
	mk_node		*mk;					// the list itself
#endif

public:
#ifdef ANN_REALTIME
	ANNmin_k(int max=0)					// constructor (given max size)
		{
			reset(max);
		}

	void	reset(int max)
		{
			n = 0;						// initially no items
			k = max;					// maximum number of items
		}
	virtual ~ANNmin_k() {}				// destructor

#else
	ANNmin_k(int max)					// constructor (given max size)
		{
			n = 0;						// initially no items
			k = max;					// maximum number of items
			mk = new mk_node[max+1];	// sorted array of keys
		}

	virtual ~ANNmin_k()					// destructor
		{ delete [] mk; }
#endif

	PQKkey ANNmin_key()					// return minimum key
		{ return (n > 0 ? mk[0].key : PQ_NULL_KEY); }
	
	PQKkey max_key()					// return maximum key
		{ return (n == k ? mk[k-1].key : PQ_NULL_KEY); }
	
	PQKkey ith_smallest_key(int i)		// ith smallest key (i in [0..n-1])
		{ return (i < n ? mk[i].key : PQ_NULL_KEY); }
	
	PQKinfo ith_smallest_info(int i)	// info for ith smallest (i in [0..n-1])
		{ return (i < n ? mk[i].info : PQ_NULL_INFO); }

	inline void insert(					// insert item (inlined for speed)
		PQKkey kv,						// key value
		PQKinfo inf)					// item info
		{
			register int i;
										// slide larger values up
			for (i = n; i > 0; i--) {
				if (mk[i-1].key > kv)
					mk[i] = mk[i-1];
				else
					break;
			}
			mk[i].key = kv;				// store element here
			mk[i].info = inf;
			if (n < k) n++;				// increment number of items
			ANN_FLOP(k-i+1)				// increment floating ops
		}

	ANN_RT_DESTROY(ANNmin_k)
};

#endif
