cmake_minimum_required(VERSION 2.8.3)
project(datasettools)

set(CMAKE_BUILD_TYPE "RelWithDebInfo") # Release
set(CMAKE_CXX_FLAGS "-std=c++11")
#-DEIGEN_USE_MKL_ALL")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rosbag
    std_msgs
    geometry_msgs
    nav_msgs
    # mav_msgs
    quadrotor_msgs
    quadrotor_common
    tf
    cv_bridge
    uavmotor
    )

set(OpenCV_DIR /usr/local/share/OpenCV)
find_package(OpenCV REQUIRED)

message(WARNING "OpenCV_VERSION: ${OpenCV_VERSION}")

# find_package(Ceres REQUIRED)

include_directories(${catkin_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS})

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

catkin_package()

add_executable(timeconvert
    src/timeconvert.cpp
    )

target_link_libraries(timeconvert ${catkin_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES}) 


add_executable(rpmconvert
    src/rpmconvert.cpp
    )
target_link_libraries(rpmconvert ${catkin_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES}) 




