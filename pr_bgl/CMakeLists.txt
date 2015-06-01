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

add_executable(test_heap src/test_heap.cpp)
add_executable(test_incbi src/test_incbi.cpp)
