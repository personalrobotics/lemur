cmake_minimum_required(VERSION 2.8.3)
project(checkmask)
find_package(catkin REQUIRED COMPONENTS roscpp)
find_package(OMPL REQUIRED)
find_package(Boost REQUIRED)
catkin_package(
   INCLUDE_DIRS include)
set(CMAKE_BUILD_TYPE Release)
include_directories(
include
   ${catkin_INCLUDE_DIRS}
   ${Boost_INCLUDE_DIR}
   ${OMPL_INCLUDE_DIRS}
   )
link_libraries(
   ${catkin_LIBRARIES}
   ${OMPL_LIBRARIES}
   rt # for clock_gettime
   )
add_library(${PROJECT_NAME} src/graph.cpp)
add_executable(graphtest src/graphtest.cpp)
target_link_libraries(graphtest ${PROJECT_NAME})
add_executable(dijk src/dijk.cpp)
install(TARGETS ${PROJECT_NAME} graphtest dijk
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
