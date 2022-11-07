#!/bin/bash
set -e  # halt on first error
source "config.bash"

apt_install cmake libeigen3-dev libv4l-dev libopencv-*
build_cmake_project apriltag
