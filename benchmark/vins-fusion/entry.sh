#!/bin/bash
export LD_LIBRARY_PATH=/usr/local/lib
source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash
exec "$@"
