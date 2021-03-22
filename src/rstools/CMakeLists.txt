cmake_minimum_required(VERSION 2.8.3)
project(rstools)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11 -march=native -DEIGEN_DONT_PARALLELIZE")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    message_generation
    )

# find_package(catkin REQUIRED genmsg actionlib_msgs actionlib)

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


add_executable(syncmode src/syncmode.cpp)
target_link_libraries(syncmode ${catkin_LIBRARIES} )


add_dependencies(syncmode ${catkin_EXPORTED_TARGETS})
target_link_libraries(
    syncmode
    realsense2
    ${catkin_LIBRARIES}
)

