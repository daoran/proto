#!/bin/bash
set -eu
# make build proto_ros2
cd ~/px4_ws && colcon build
