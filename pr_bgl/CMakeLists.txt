cmake_minimum_required(VERSION 2.8.3)
project(pr_bgl)

find_package(catkin REQUIRED)
find_package(Boost REQUIRED)

catkin_package(
   INCLUDE_DIRS include
)

include_directories(
   include
   ${Boost_INCLUDE_DIRS})

#set(CMAKE_BUILD_TYPE Release)

add_executable(test_incbi src/test_incbi.cpp)

if (CATKIN_ENABLE_TESTING)
   catkin_add_gtest(test_heap_indexed test/heap_indexed.cpp)
   catkin_add_gtest(test_partition_all test/partition_all.cpp)
   catkin_add_gtest(test_partition_simple test/partition_simple.cpp)
endif ()
