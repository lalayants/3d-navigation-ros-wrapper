cmake_minimum_required(VERSION 2.8.3 FATAL_ERROR)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

project(3d_navigation_ros_wrapper)

find_package(catkin REQUIRED COMPONENTS roscpp 3d_navigation)


# Finding planning mapping and math packages
find_package(octomap REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(ompl REQUIRED)
find_package(fcl REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${OMPL_INCLUDE_DIRS} ${fcl_INCLUDE_DIRS})
include_directories(include/${CMAKE_PROJECT_NAME})

add_library(global_planner_3d_ros_wrapper SHARED src/global_planner_wrapper.cpp)
target_link_libraries(global_planner_3d_ros_wrapper global_planner_3d ${OCTOMAP_LIBRARIES} ${catkin_LIBRARIES} fcl ${OMPL_LIBRARIES})

## Main script for example and tests
# add_executable(global_planner_node src/global_planner_node.cpp)
# target_link_libraries(global_planner_node global_planner_3d_ros_wrapper)