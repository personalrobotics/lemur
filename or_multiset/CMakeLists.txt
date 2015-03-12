cmake_minimum_required(VERSION 2.8.3)
project(or_multiset)
find_package(catkin REQUIRED COMPONENTS ompl_multiset)
find_package(OMPL REQUIRED)
find_package(OpenRAVE REQUIRED)

catkin_package()
catkin_add_env_hooks("20.${PROJECT_NAME}"
   SHELLS sh
   DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/catkin-env-hooks"
)

set(CMAKE_BUILD_TYPE Release)
include_directories(
   ${catkin_INCLUDE_DIRS}
   ${OMPL_INCLUDE_DIRS}
   ${OpenRAVE_INCLUDE_DIRS})
link_libraries(
   ${catkin_LIBRARIES}
   ${OMPL_LIBRARIES}
   ${OpenRAVE_LIBRARIES}
   ${OpenRAVE_CORE_LIBRARIES}
   rt) # for clock_gettime

# openrave bindings
add_definitions("-fno-strict-aliasing -Wall")
add_library(${PROJECT_NAME} MODULE src/plugin.cpp src/planner_checkmask.cpp)
set_target_properties(${PROJECT_NAME} PROPERTIES
   COMPILE_FLAGS "${OpenRAVE_CXX_FLAGS} -Wno-unknown-pragmas"
   LINK_FLAGS "${OpenRAVE_LINK_FLAGS}"
   PREFIX ""
   LIBRARY_OUTPUT_DIRECTORY
   "${CATKIN_DEVEL_PREFIX}/lib/openrave-${OpenRAVE_LIBRARY_SUFFIX}"
)

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION "lib/openrave-${OpenRAVE_LIBRARY_SUFFIX}"
)
