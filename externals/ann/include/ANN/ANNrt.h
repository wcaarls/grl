/*
 * ANNrt.h
 *
 *  Created on: Aug 17, 2010
 *      Author: Erik Schuitema
 */

#ifndef ANNRT_H_
#define ANNRT_H_

//#define ann_new new
#ifndef ANN_REALTIME
	#define ann_new(T) new T
	#define ann_new_arr(T, N) new T[N]
	#define ann_delete(T) delete T
	#define ann_delete_obj(T) delete T
	#define ann_delete_arr(T) delete[] T
	#define ann_delete_obj_arr(T, N) delete[] T;

	#define ANN_RT_DESTROY(T)

#else
	#include <MemoryPool.h>

	// Memory pool for real-time ANN
	extern CMemoryPool* gAnnMemoryPool;
	void annSetRealtimeMemoryPool(CMemoryPool* pool);

	// Definition of ann_new and ann_delete
	#define ann_new(T) new(gAnnMemoryPool->alloc(sizeof(T))) T
	#define ann_new_arr(T, N) new(gAnnMemoryPool->alloc(sizeof(T)*N)) T[N]
	#define ann_delete(T) gAnnMemoryPool->dealloc(T)
	// Use ann_delete_obj(T) for class types. This explicitly calls the destructor (use ANN_RT_DESTROY, see below)
	#define ann_delete_obj(T) do {T->ann_destroy(); gAnnMemoryPool->dealloc(T);} while (0)
	#define ann_delete_arr(T) gAnnMemoryPool->dealloc(T)
	#define ann_delete_obj_arr(T, N) do {for (int i=N-1; i>=0; i--) T[i].ann_destroy(); gAnnMemoryPool->dealloc(T);} while (0)

	// Define a macro that you can add to your class to explicitly call the destructor
	#define ANN_RT_DESTROY(T) inline virtual void ann_destroy(){this->~T();}


#endif




#endif /* ANNRT_H_ */
