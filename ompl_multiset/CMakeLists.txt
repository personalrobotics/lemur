cmake_minimum_required(VERSION 2.8.3)
project(ompl_multiset)

find_package(catkin REQUIRED COMPONENTS pr_bgl)
find_package(OMPL REQUIRED)
find_package(Boost REQUIRED)
find_package(OpenSSL REQUIRED)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
)

set(CMAKE_BUILD_TYPE Release)

include_directories(
   include
   ${catkin_INCLUDE_DIRS}
   ${OMPL_INCLUDE_DIRS}
   ${Boost_INCLUDE_DIR}
   ${OPENSSL_INCLUDE_DIR})
link_libraries(
   ${catkin_LIBRARIES}
   ${OMPL_LIBRARIES}
   ${OPENSSL_LIBRARIES})

#add_definitions("-Wall -Werror")
add_definitions("-Wall")

add_library(${PROJECT_NAME}
   src/Cache.cpp
   src/MultiSetPRM.cpp
   src/E8Roadmap.cpp
   src/RoadmapSampledConst.cpp
   src/RoadmapSampledDensified.cpp
   src/SamplerGenMonkeyPatch.cpp
   src/SpaceID.cpp
   src/util.cpp)

add_executable(generate-unit-roadmap src/generate_unit_roadmap.cpp)
target_link_libraries(generate-unit-roadmap ${PROJECT_NAME})

add_executable(test-graph-overlay src/test_graph_overlay.cpp)
target_link_libraries(test-graph-overlay ${PROJECT_NAME})

catkin_add_gtest(roadmapgen test/roadmapgen.cpp)
target_link_libraries(roadmapgen ${PROJECT_NAME})

catkin_add_gtest(logic test/logic.cpp)

catkin_add_gtest(family_effort_model test/family_effort_model.cpp)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
