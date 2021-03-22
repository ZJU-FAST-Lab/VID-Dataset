cmake_minimum_required(VERSION 2.8.3)
project(uavmotor)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11 -march=native -DEIGEN_DONT_PARALLELIZE")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    message_generation
    serial
    # quadrotor_msgs
    # quadrotor_common
    nav_msgs
    )

# find_package(catkin REQUIRED genmsg actionlib_msgs actionlib)
add_message_files(
  FILES
  m3508.msg
  motordata.msg
  pulsecap.msg  
)
generate_messages(DEPENDENCIES  std_msgs )
# generate_messages(
#     DEPENDENCIES
#     std_msgs
#     geometry_msgs
#     nav_msgs
# )


set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3  REQUIRED)
#find_package(yaml-cpp  REQUIRED)
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    #${YAML_CPP_INCLUDE_DIR}
    )

catkin_package(
    )


add_executable(m100withm3508 src/m100withm3508.cpp)
target_link_libraries(m100withm3508 ${catkin_LIBRARIES} )
