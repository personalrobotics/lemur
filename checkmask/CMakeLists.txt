cmake_minimum_required(VERSION 2.8.3)
project(checkmask)
find_package(catkin REQUIRED COMPONENTS roscpp)
find_package(OMPL REQUIRED)
find_package(Boost REQUIRED)
find_package(OpenRAVE REQUIRED)
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME})
set(CMAKE_BUILD_TYPE Release)
include_directories(
   include
   ${catkin_INCLUDE_DIRS}
   ${OMPL_INCLUDE_DIRS}
   ${Boost_INCLUDE_DIR}
   ${OpenRAVE_INCLUDE_DIRS}
   )
link_libraries(
   ${catkin_LIBRARIES}
   ${OMPL_LIBRARIES}
   ${OpenRAVE_LIBRARIES}
   ${OpenRAVE_CORE_LIBRARIES}
   rt # for clock_gettime
   )

# planner
add_library(${PROJECT_NAME} src/graph.cpp)

# testers
add_executable(test2d src/tests/test2d.cpp)
target_link_libraries(test2d ${PROJECT_NAME})

add_executable(test2dimg src/tests/test2dimg.cpp)
target_link_libraries(test2dimg ${PROJECT_NAME} png)

add_executable(testherb src/tests/testherb.cpp)
target_link_libraries(testherb ${PROJECT_NAME})

add_executable(dijk src/tests/dijk.cpp)

# openrave bindings
add_definitions("-fno-strict-aliasing -Wall")
add_library(or${PROJECT_NAME} MODULE src/orompl/plugin.cpp src/orompl/planner_checkmask.cpp)
target_link_libraries(or${PROJECT_NAME} ${PROJECT_NAME})
set_target_properties(or${PROJECT_NAME} PROPERTIES
   COMPILE_FLAGS "${OpenRAVE_CXX_FLAGS} -Wno-unknown-pragmas"
   LINK_FLAGS "${OpenRAVE_LINK_FLAGS}"
   PREFIX ""
   LIBRARY_OUTPUT_DIRECTORY
   "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_LIB_DESTINATION}/openrave-${OpenRAVE_LIBRARY_SUFFIX}"
)

install(TARGETS ${PROJECT_NAME} test2d dijk
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
