#!/bin/bash
source devel/setup.bash
# roscore & sleep 3
roslaunch datasettools timeconvert.launch & sleep 3
roslaunch datasettools rpmconvert.launch & sleep 3
rosbag play $1







