cmake_minimum_required(VERSION 2.8.3)
project(quadrotor_common)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

add_definitions(-std=c++11)

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS std_msgs sensor_msgs geometry_msgs nav_msgs roscpp quadrotor_msgs)

include_directories(
  include
  include/quadrotor_common
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR})

catkin_package(
  INCLUDE_DIRS include
  # LIBRARIES quadrotor_common
  # CATKIN_DEPENDS std_msgs sensor_msgs geometry_msgs nav_msgs roscpp
) 
  
add_library(${PROJECT_NAME} 
  src/control_command.cpp
  src/geometry_eigen_conversions.cpp
  src/math_common.cpp
  src/parameter_helper.cpp
  src/quad_state_estimate.cpp
  src/quaternion_functions.cpp
  src/trajectory_point.cpp
  src/trajectory.cpp)
add_dependencies(${PROJECT_NAME} 
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} 
  ${catkin_LIBRARIES} 
  ${EIGEN_LIBS})


