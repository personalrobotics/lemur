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
   
   catkin_add_gtest(test_heap_indexed test/test_heap_indexed.cpp)
   
   catkin_add_gtest(test_incbi test/test_incbi.cpp)
   
   catkin_add_gtest(test_lazysp test/test_lazysp.cpp)
   set_target_properties(test_lazysp PROPERTIES
      COMPILE_FLAGS -DDATADIR="${PROJECT_SOURCE_DIR}/test/data")
   target_link_libraries(test_lazysp ${Boost_LIBRARIES})
   
   catkin_add_gtest(test_lpastar test/test_lpastar.cpp)
   set_target_properties(test_lpastar PROPERTIES
      COMPILE_FLAGS -DDATADIR="${PROJECT_SOURCE_DIR}/test/data")
   target_link_libraries(test_lpastar ${Boost_LIBRARIES})
   
   catkin_add_gtest(test_partition_all test/test_partition_all.cpp)
   
   catkin_add_gtest(test_partition_simple test/test_partition_simple.cpp)

endif ()
