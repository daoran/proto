#!/bin/bash
set -e

. /opt/ros/humble/setup.sh \
  && mkdir -p $HOME/proto_ws \
  && cd $HOME/proto_ws && colcon build --parallel-workers 4 \
  && . install/setup.sh \
  && ros2 launch proto_ros2 calib_sim.launch.py
