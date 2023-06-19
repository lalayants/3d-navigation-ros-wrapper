cmake_minimum_required(VERSION 2.8.3 FATAL_ERROR)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

project(3d_navigation_ros_wrapper)

find_package(catkin REQUIRED COMPONENTS)
# catkin_package(
# 	CATKIN_DEPENDS 3d_navigation roscpp pr2_msgs tf control_msgs diagnostic_msgs geometry_msgs nav_msgs std_msgs std_srvs trajectory_msgs
# )

# Finding planning mapping and math packages
find_package(global_planner_3d REQUIRED)
find_package(octomap REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} ${OMPL_INCLUDE_DIRS} ${fcl_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(include/${CMAKE_PROJECT_NAME})

## Main script for example and tests
add_executable(main src/main.cpp)
target_link_libraries(main global_planner_3d)