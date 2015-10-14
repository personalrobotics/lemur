cmake_minimum_required(VERSION 2.8.3)
project(ompl_multiset)

find_package(catkin REQUIRED COMPONENTS pr_bgl)
find_package(OMPL REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem graph program_options system)
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
   ${Boost_LIBRARIES}
   ${OPENSSL_LIBRARIES})

#add_definitions("-Wall -Werror")
add_definitions("-Wall")

add_library(${PROJECT_NAME}
   src/Cache.cpp
   src/MultiSetPRM.cpp
   src/E8Roadmap.cpp
   src/MultiSetRoadmapSampledConst.cpp
   src/MultiSetRoadmapSampledDensified.cpp
   src/SamplerGenMonkeyPatch.cpp
   src/SpaceID.cpp
   src/util.cpp)

add_executable(generate-unit-roadmap src/generate_unit_roadmap.cpp)
target_link_libraries(generate-unit-roadmap ${PROJECT_NAME})

add_executable(test_graph_overlay src/test_graph_overlay.cpp)
target_link_libraries(test_graph_overlay ${PROJECT_NAME})

if (CATKIN_ENABLE_TESTING)

   catkin_add_gtest(test_roadmapgen test/roadmapgen.cpp)
   target_link_libraries(test_roadmapgen ${PROJECT_NAME})

   catkin_add_gtest(test_logic test/logic.cpp)

   catkin_add_gtest(test_family_effort_model test/family_effort_model.cpp)

   catkin_add_gtest(test_e8simple test/e8simple.cpp)
   target_link_libraries(test_e8simple ${PROJECT_NAME})

   catkin_add_gtest(test_e8fromfile test/e8fromfile.cpp)
   set_target_properties(test_e8fromfile PROPERTIES COMPILE_FLAGS -DDATADIR="${PROJECT_SOURCE_DIR}/test/data")
   target_link_libraries(test_e8fromfile ${PROJECT_NAME})

endif ()

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
