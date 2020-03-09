#!/bin/bash
set -e  # halt on first error
BASEDIR=$(dirname "$0")
source "$BASEDIR/config.bash"

apt_install \
  liboctomap-dev \
  liboctovis-dev \
  ros-melodic-octomap-*
