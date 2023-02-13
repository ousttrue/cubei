/*
	Copyright (c) 2012 Hiroshi Matsuike

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

	2. Altered source versions must be plainly marked as such, and must not be
	misrepresented as being the original software.

	3. This notice may not be removed or altered from any source distribution.
*/

#ifndef EPX_SORT_H
#define EPX_SORT_H

#include "../EpxBase.h"

#define Key(a) ((a).key)

namespace EasyPhysics {

#ifndef EPX_DOXYGEN_SKIP

template <class SortData>
void epxMergeTwoBuffers(SortData* d1,unsigned int n1,SortData* d2,unsigned int n2,SortData *buff)
{
	unsigned int i=0,j=0;

	while(i<n1&&j<n2) {
		if(Key(d1[i]) < Key(d2[j])) {
			buff[i+j] = d1[i++];
		}
		else {
			buff[i+j] = d2[j++];
		}
	}
	
	if(i<n1) {
		while(i<n1) {
			buff[i+j] = d1[i++];
		}
	}
	else if(j<n2) {
		while(j<n2) {
			buff[i+j] = d2[j++];
		}
	}

	for(unsigned int k=0;k<(n1+n2);k++) {
		d1[k] = buff[k];
	}
}

#endif // EPX_DOXYGEN_SKIP

/// ソート
/// @param[in,out] d ソートするデータの配列
/// @param buff ソート用のバッファ（入力データと同サイズ）
/// @param n データの数
template <class SortData>
void epxSort(SortData *d,SortData *buff,int n)
{
	int n1 = n>>1;
	int n2 = n-n1;
	if(n1>1) epxSort(d,buff,n1);
	if(n2>1) epxSort(d+n1,buff,n2);
	epxMergeTwoBuffers(d,n1,d+n1,n2,buff);
}

} // namespace EasyPhysics

#endif // EPX_SORT_H
