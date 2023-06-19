cmake_minimum_required(VERSION 2.8.3 FATAL_ERROR)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

project(3d_navigation)

# Finding planning mapping and math packages
find_package(PCL REQUIRED COMPONENTS common io)
find_package(ompl REQUIRED)
find_package(fcl REQUIRED)
find_package(octomap REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} ${OMPL_INCLUDE_DIRS} ${fcl_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(include/${CMAKE_PROJECT_NAME})

add_library(global_planner_3d SHARED src/global_planner.cpp)
target_link_libraries(global_planner_3d fcl ${OCTOMAP_LIBRARIES} ${OMPL_LIBRARIES})

## Main script for example and tests
add_executable(main src/main.cpp)
target_link_libraries(main global_planner_3d)

file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/maps
    DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
    

