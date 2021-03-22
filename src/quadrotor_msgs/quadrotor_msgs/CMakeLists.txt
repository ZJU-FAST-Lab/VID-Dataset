cmake_minimum_required(VERSION 2.8.3)
project(quadrotor_msgs)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

add_definitions(-std=c++11)

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS std_msgs sensor_msgs geometry_msgs nav_msgs roscpp message_generation)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR})

add_message_files(
    FILES
    AutopilotFeedback.msg
    ControlCommand.msg
    LowLevelFeedback.msg
    Trajectory.msg
    TrajectoryPoint.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
    nav_msgs
    sensor_msgs
)

catkin_package(
  # CATKIN_DEPENDS std_msgs sensor_msgs geometry_msgs nav_msgs roscpp message_runtime
)