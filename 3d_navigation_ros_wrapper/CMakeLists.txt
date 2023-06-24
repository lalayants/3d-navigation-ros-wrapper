cmake_minimum_required(VERSION 2.8.3 FATAL_ERROR)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

project(3d_navigation_ros_wrapper)

find_package(catkin REQUIRED COMPONENTS 3d_navigation roscpp)
find_package(fcl)

catkin_package(
  LIBRARIES global_planner_3d_ros_wrapper
  CATKIN_DEPENDS roscpp
)

include_directories(include/${CMAKE_PROJECT_NAME})
include_directories(${catkin_INCLUDE_DIRS} ${3d_navigation_INCLUDE_DIRS} ${fcl_INCLUDE_DIRS})

add_library(global_planner_3d_ros_wrapper SHARED src/global_planner_wrapper.cpp)
target_link_libraries(global_planner_3d_ros_wrapper global_planner_3d ${OCTOMAP_LIBRARIES} ${catkin_LIBRARIES} fcl ${OMPL_LIBRARIES})

## Main script for example and tests
# add_executable(main src/main.cpp)
# target_link_libraries(main ${catkin_LIBRARIES} ${3d_navigation_LIBRARIES} fcl)