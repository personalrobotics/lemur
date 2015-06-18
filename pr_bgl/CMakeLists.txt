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

foreach(testname heap incbi sebc sebc_dump)
   add_executable(test_${testname} src/test_${testname}.cpp)
endforeach()
