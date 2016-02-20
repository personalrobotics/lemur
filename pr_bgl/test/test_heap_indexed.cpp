/* File: heap_indexed.cpp
 * Author: Chris Dellin <cdellin@gmail.com>
 * Copyright: 2015 Carnegie Mellon University
 * License: BSD
 */

#include <cstdio>
#include <cstdlib>
#include <vector>
#include <pr_bgl/heap_indexed.h>

#include <gtest/gtest.h>

TEST(HeapIndexedTestCase, HeapIndexedTest)
{
   pr_bgl::heap_indexed<double> h;
   h.insert(0, 5.0);
   h.insert(1, 6.0);
   h.insert(2, 7.0);
   h.insert(3, 4.0);
   h.insert(4, 3.0);
   h.insert(5, 8.0);
   h.insert(6, 8.0);
   h.insert(7, 2.0);
   h.print();
   
   ASSERT_EQ(2.0, h.top_key());
   ASSERT_EQ(7, h.top_idx());
   
   h.remove(2);
   h.print();
   
   ASSERT_EQ(2.0, h.top_key());
   ASSERT_EQ(7, h.top_idx());
   
   h.remove(7);
   h.print();
   
   ASSERT_EQ(3.0, h.top_key());
   ASSERT_EQ(4, h.top_idx());
}

int main(int argc, char **argv)
{
   testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}
