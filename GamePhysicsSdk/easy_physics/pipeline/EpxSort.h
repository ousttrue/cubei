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

        2. Altered source versions must be plainly marked as such, and must not
   be misrepresented as being the original software.

        3. This notice may not be removed or altered from any source
   distribution.
*/

#ifndef EPX_SORT_H
#define EPX_SORT_H

#include "../EpxBase.h"
#include <span>

#define Key(a) ((a).key)

namespace EasyPhysics {

#ifndef EPX_DOXYGEN_SKIP

template <class SortData>
void epxMergeTwoBuffers(std::span<SortData> d1, std::span<SortData> d2,
                        std::span<SortData> buff) {
  unsigned int i = 0, j = 0;

  while (i < d1.size() && j < d2.size()) {
    if (Key(d1[i]) < Key(d2[j])) {
      buff[i + j] = d1[i];
      i++;
    } else {
      buff[i + j] = d2[j];
      j++;
    }
  }

  if (i < d1.size()) {
    while (i < d1.size()) {
      buff[i + j] = d1[i];
      i++;
    }
  } else if (j < d2.size()) {
    while (j < d2.size()) {
      buff[i + j] = d2[j];
      j++;
    }
  }

  unsigned int k = 0;
  if (d1.size()) {
    for (auto it = d1.begin(); it != d1.end(); ++k, ++it) {
      *it = buff[k];
    }
  }
  if (d2.size()) {
    for (auto it = d2.begin(); it != d2.end(); ++k, ++it) {
      *it = buff[k];
    }
  }
}

#endif // EPX_DOXYGEN_SKIP

/// ソート
/// @param[in,out] d ソートするデータの配列
/// @param buff ソート用のバッファ（入力データと同サイズ）
/// @param n データの数
template <class SortData>
void epxSort(std::span<SortData> d, std::span<SortData> buff) {
  int n1 = d.size() >> 1;
  int n2 = d.size() - n1;
  if (n1 > 1)
    epxSort(d.subspan(0, n1), buff);
  if (n2 > 1)
    epxSort(d.subspan(n1, n2), buff);
  epxMergeTwoBuffers(d.subspan(0, n1), d.subspan(n1, n2), buff);
}

} // namespace EasyPhysics

#endif // EPX_SORT_H
