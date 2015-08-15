/* File: test_heap.cpp
 * Author: Chris Dellin <cdellin@gmail.com>
 * Copyright: 2015 Carnegie Mellon University
 * License: BSD
 */

#include <cstdio>
#include <cstdlib>
#include <vector>
#include <pr_bgl/heap_indexed.h>

int main()
{
   pr_bgl::HeapIndexed<double> h;
   h.insert(0, 5.0);
   h.insert(1, 6.0);
   h.insert(2, 7.0);
   h.insert(3, 4.0);
   h.insert(4, 3.0);
   h.insert(5, 8.0);
   h.insert(6, 8.0);
   h.insert(7, 2.0);
   h.print();
   
   h.remove(2);
   h.print();
   
   return 0;
}
