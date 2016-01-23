cmake_minimum_required(VERSION 2.8.3)
project(pr_bgl)

find_package(catkin REQUIRED)
find_package(Boost REQUIRED COMPONENTS graph)

catkin_package(
   INCLUDE_DIRS include
)

include_directories(
   include
   ${Boost_INCLUDE_DIRS})

set(CMAKE_BUILD_TYPE Release)

if (CATKIN_ENABLE_TESTING)
   
   catkin_add_gtest(test_heap_indexed test/heap_indexed.cpp)
   
   catkin_add_gtest(test_lazysp test/lazysp.cpp)
   set_target_properties(test_lazysp PROPERTIES
      COMPILE_FLAGS -DDATADIR="${PROJECT_SOURCE_DIR}/test/data")
   target_link_libraries(test_lazysp ${Boost_LIBRARIES})
   
   catkin_add_gtest(test_lifelong_planning_astar test/lifelong_planning_astar.cpp)
   set_target_properties(test_lifelong_planning_astar PROPERTIES
      COMPILE_FLAGS -DDATADIR="${PROJECT_SOURCE_DIR}/test/data")
   target_link_libraries(test_lifelong_planning_astar ${Boost_LIBRARIES})
   
   catkin_add_gtest(test_inc_bi test/inc_bi.cpp)
   
   catkin_add_gtest(test_partition_all test/partition_all.cpp)
   
   catkin_add_gtest(test_partition_simple test/partition_simple.cpp)

endif ()
