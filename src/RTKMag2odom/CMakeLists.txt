cmake_minimum_required(VERSION 2.8.3)

project(rtkmag2odom)

set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -fPIC")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  nav_msgs
)

find_package(Boost REQUIRED COMPONENTS system thread)
find_package (GeographicLib REQUIRED)
find_package(Eigen3  REQUIRED)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${GeographicLib_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    include
)
add_definitions(${GeographicLib_DEFINITIONS})

catkin_package(
    INCLUDE_DIRS include
)

add_executable(rtkmag2odom_node src/rtkmag2odom_node.cpp src/rtkmag2odom.cpp)

target_link_libraries(rtkmag2odom_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${GeographicLib_LIBRARIES})