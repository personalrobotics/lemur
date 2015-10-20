cmake_minimum_required(VERSION 2.8.3)

project(test_multiset)

find_package(catkin REQUIRED COMPONENTS ompl_multiset)
find_package(OMPL REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(OpenRAVE REQUIRED)

catkin_package()

#set(CMAKE_BUILD_TYPE Release)

include_directories(
   ${catkin_INCLUDE_DIRS}
   ${OMPL_INCLUDE_DIRS}
   ${Boost_INCLUDE_DIR}
   ${OpenRAVE_INCLUDE_DIRS})
link_libraries(
   ${catkin_LIBRARIES}
   ${OMPL_LIBRARIES}
   ${Boost_LIBRARIES}
   ${OpenRAVE_LIBRARIES}
   ${OpenRAVE_CORE_LIBRARIES}
   rt) # for clock_gettime

add_definitions("-Wall -Werror")

add_executable(test2d src/test2d.cpp)

add_executable(test2dimg src/test2dimg.cpp)
target_link_libraries(test2dimg png)

add_executable(testherb src/testherb.cpp)
target_link_libraries(testherb)

add_executable(dijk src/dijk.cpp)

add_executable(roadmaps-2d src/roadmaps-2d.cpp)

install(TARGETS test2d test2dimg testherb dijk
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
