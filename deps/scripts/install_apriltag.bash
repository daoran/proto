#!/bin/bash
set -e  # halt on first error
source "config.bash"
REPO_URL=https://github.com/chutsu/apriltag

apt_install cmake libeigen3-dev libv4l-dev libopencv-*
install_git_repo $REPO_URL apriltag
