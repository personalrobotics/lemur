cmake_minimum_required(VERSION 2.8.3)
project(or_lemur)

include(CheckCXXSourceCompiles)

find_package(Boost REQUIRED COMPONENTS thread)
find_package(catkin REQUIRED COMPONENTS pr_bgl ompl_lemur)
find_package(OMPL REQUIRED)
find_package(OpenRAVE REQUIRED)

# 1.2.0 ompl (ROS kinetic) switched from boost to std smart pointers
set(CMAKE_REQUIRED_INCLUDES ${OMPL_INCLUDE_DIRS})
check_cxx_source_compiles(
    "#include <ompl/base/StateSpace.h>
    int main(){ ompl::base::StateSpacePtr s = boost::shared_ptr<ompl::base::StateSpace>(); }"
    OR_LEMUR_HAS_BOOSTSMARTPTRS)
configure_file(
    include/${PROJECT_NAME}/config.h.in
    ${CATKIN_DEVEL_PREFIX}/include/${PROJECT_NAME}/config.h
)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
)
catkin_add_env_hooks("20.${PROJECT_NAME}"
   SHELLS sh
   DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/catkin-env-hooks"
)

set(CMAKE_BUILD_TYPE Release)

if(OR_LEMUR_HAS_BOOSTSMARTPTRS)
else(OR_LEMUR_HAS_BOOSTSMARTPTRS)
add_definitions(-std=c++11)
endif(OR_LEMUR_HAS_BOOSTSMARTPTRS)

include_directories(
   include
   ${CATKIN_DEVEL_PREFIX}/include
   ${catkin_INCLUDE_DIRS}
   ${OMPL_INCLUDE_DIRS}
   ${OpenRAVE_INCLUDE_DIRS})
link_libraries(
   ${catkin_LIBRARIES}
   ${OMPL_LIBRARIES}
   ${OpenRAVE_LIBRARIES}
   ${OpenRAVE_CORE_LIBRARIES}
   rt) # for clock_gettime

add_definitions("-Wall -Werror -Wno-maybe-uninitialized")
add_definitions("-fno-strict-aliasing")

add_library(${PROJECT_NAME}
   src/or_ompl_conversions.cpp
   src/module_family.cpp
   src/parse_args.cpp
   src/planner_lemur.cpp
   src/planner_family.cpp
   src/planner_cctimer.cpp
   src/SemiTorusStateSpace.cpp
   )

# openrave bindings
add_library(${PROJECT_NAME}_orplugin MODULE src/plugin.cpp)
target_link_libraries(${PROJECT_NAME}_orplugin ${PROJECT_NAME})
set_target_properties(${PROJECT_NAME}_orplugin PROPERTIES
   COMPILE_FLAGS "${OpenRAVE_CXX_FLAGS} -Wno-unknown-pragmas"
   LINK_FLAGS "${OpenRAVE_LINK_FLAGS}"
   OUTPUT_NAME "${PROJECT_NAME}"
   PREFIX ""
   LIBRARY_OUTPUT_DIRECTORY
   "${CATKIN_DEVEL_PREFIX}/lib/openrave-${OpenRAVE_LIBRARY_SUFFIX}")

if (CATKIN_ENABLE_TESTING)
   
   catkin_add_gtest(test_wam7 test/cpp/test_wam7.cpp)
   target_link_libraries(test_wam7
     ${OpenRAVE_LIBRARIES}
     ${OpenRAVE_CORE_LIBRARIES}
     ${Boost_LIBRARIES}
   )
    
   catkin_add_gtest(test_module_family test/cpp/test_module_family.cpp)
   target_link_libraries(test_module_family
     ${PROJECT_NAME}
     ${OpenRAVE_LIBRARIES}
     ${OpenRAVE_CORE_LIBRARIES}
     ${Boost_LIBRARIES}
   )
   
   catkin_add_nosetests(test/python/wam7.py)
   
endif()

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION "lib/openrave-${OpenRAVE_LIBRARY_SUFFIX}"
)
