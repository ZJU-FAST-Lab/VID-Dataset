cmake_minimum_required(VERSION 2.8.3)
project(state_predictor)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

add_definitions(-std=c++11)

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS std_msgs sensor_msgs geometry_msgs nav_msgs roscpp quadrotor_msgs quadrotor_common)

include_directories(
  include
  include/state_predictor
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR})

catkin_package(
  INCLUDE_DIRS include
  # CATKIN_DEPENDS std_msgs sensor_msgs geometry_msgs nav_msgs roscpp quadrotor_msgs quadrotor_common
  # DEPENDS quadrotor_common
)

add_library(${PROJECT_NAME} 
  src/state_predictor.cpp 
  src/quad_ext_state_estimate.cpp)
add_dependencies(${PROJECT_NAME} 
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS} 
  quadrotor_msgs_gencpp)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES} 
  ${EIGEN_LIBS})
