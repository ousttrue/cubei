//--------------------------------------------------------------------------------------------------
/**
@file	q3Memory.h

@author	Randy Gaul
@date	10/10/2014

	Copyright (c) 2014 Randy Gaul http://www.randygaul.net

	This software is provided 'as-is', without any express or implied
	warranty. In no event will the authors be held liable for any damages
	arising from the use of this software.

	Permission is granted to anyone to use this software for any purpose,
	including commercial applications, and to alter it and redistribute it
	freely, subject to the following restrictions:
	  1. The origin of this software must not be misrepresented; you must not
	     claim that you wrote the original software. If you use this software
	     in a product, an acknowledgment in the product documentation would be
	     appreciated but is not required.
	  2. Altered source versions must be plainly marked as such, and must not
	     be misrepresented as being the original software.
	  3. This notice may not be removed or altered from any source distribution.
*/
//--------------------------------------------------------------------------------------------------

#ifndef Q3MEMORY_H
#define Q3MEMORY_H

#include <stdlib.h>

#include "q3Types.h"

//--------------------------------------------------------------------------------------------------
// Memory Macros
//--------------------------------------------------------------------------------------------------
inline void* q3Alloc( int bytes )
{
	return malloc( bytes );
}

inline void q3Free( void* memory )
{
	free( memory );
}

#define Q3_PTR_ADD( P, BYTES ) \
	((decltype( P ))(((uint8_t *)P) + (BYTES)))

//--------------------------------------------------------------------------------------------------
// q3Stack
//--------------------------------------------------------------------------------------------------

class q3Stack
{
private:
	struct q3StackEntry
	{
		uint8_t *data;
		int size;
	};

public:
	q3Stack( );
	~q3Stack( );

	void Reserve( uint32_t size );
	void *Allocate( int size );
	void Free( void *data );

private:
	uint8_t* m_memory;
	q3StackEntry* m_entries;

	uint32_t m_index;

	int m_allocation;
	int m_entryCount;
	int m_entryCapacity;
	uint32_t m_stackSize;
};

//--------------------------------------------------------------------------------------------------
// q3Heap
//--------------------------------------------------------------------------------------------------
// 20 MB heap size, change as necessary
const int q3k_heapSize = 1024 * 1024 * 20;
const int q3k_heapInitialCapacity = 1024;

// Operates on first fit basis in attempt to improve cache coherency
class q3Heap
{
private:
	struct q3Header
	{
		q3Header* next;
		q3Header* prev;
		int size;
	};

	struct q3FreeBlock
	{
		q3Header* header;
		int size;
	};

public:
	q3Heap( );
	~q3Heap( );

	void *Allocate( int size );
	void Free( void *memory );

private:
	q3Header* m_memory;

	q3FreeBlock* m_freeBlocks;
	int m_freeBlockCount;
	int m_freeBlockCapacity;
};

//--------------------------------------------------------------------------------------------------
// q3PagedAllocator
//--------------------------------------------------------------------------------------------------
class q3PagedAllocator
{
	struct q3Block
	{
		q3Block* next;
	};

	struct q3Page
	{
		q3Page* next;
		q3Block* data;
	};

public:
	q3PagedAllocator( int elementSize, int elementsPerPage );
	~q3PagedAllocator( );

	void* Allocate( );
	void Free( void* data );

	void Clear( );

private:
	int m_blockSize;
	int m_blocksPerPage;

	q3Page *m_pages;
	int m_pageCount;

	q3Block *m_freeList;
};

#endif // Q3MEMORY_H
