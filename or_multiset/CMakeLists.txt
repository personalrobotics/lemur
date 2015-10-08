cmake_minimum_required(VERSION 2.8.3)
project(or_multiset)

find_package(catkin REQUIRED COMPONENTS pr_bgl ompl_multiset)
find_package(OMPL REQUIRED)
find_package(OpenRAVE REQUIRED)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
)
catkin_add_env_hooks("20.${PROJECT_NAME}"
   SHELLS sh
   DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/catkin-env-hooks"
)

set(CMAKE_BUILD_TYPE Release)

include_directories(
   include
   ${catkin_INCLUDE_DIRS}
   ${OMPL_INCLUDE_DIRS}
   ${OpenRAVE_INCLUDE_DIRS})
link_libraries(
   ${catkin_LIBRARIES}
   ${OMPL_LIBRARIES}
   ${OpenRAVE_LIBRARIES}
   ${OpenRAVE_CORE_LIBRARIES}
   rt) # for clock_gettime

add_definitions("-Wall -Werror")
add_definitions("-fno-strict-aliasing")

add_library(${PROJECT_NAME} src/inter_link_checks.cpp)

# openrave bindings
add_library(${PROJECT_NAME}_orplugin MODULE
   src/plugin/module_subset_manager.cpp
   src/plugin/planner_multiset_prm.cpp
   src/plugin/planner_e8roadmap.cpp
   src/plugin/planner_e8roadmapselfcc.cpp
   src/plugin/plugin.cpp)
target_link_libraries(${PROJECT_NAME}_orplugin ${PROJECT_NAME})
set_target_properties(${PROJECT_NAME}_orplugin PROPERTIES
   COMPILE_FLAGS "${OpenRAVE_CXX_FLAGS} -Wno-unknown-pragmas"
   LINK_FLAGS "${OpenRAVE_LINK_FLAGS}"
   OUTPUT_NAME "${PROJECT_NAME}"
   PREFIX ""
   LIBRARY_OUTPUT_DIRECTORY
   "${CATKIN_DEVEL_PREFIX}/lib/openrave-${OpenRAVE_LIBRARY_SUFFIX}")

catkin_add_gtest(test_selfcc test/selfcc.cpp)
target_link_libraries(test_selfcc ${OpenRAVE_LIBRARIES} ${OpenRAVE_CORE_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION "lib/openrave-${OpenRAVE_LIBRARY_SUFFIX}"
)
