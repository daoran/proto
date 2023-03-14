#!/bin/bash
set -e

export IGN_GAZEBO_SERVER_CONFIG_PATH=$PWD/server.config
gz sim calibration.sdf --gui-config gazebo_gui.config
# ign gazebo --gui-config gazebo_gui.config
