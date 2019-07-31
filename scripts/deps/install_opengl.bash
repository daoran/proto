#!/bin/bash
set -e  # halt on first error
BASEDIR=$(dirname "$0")
source "$BASEDIR/config.bash"

apt_install libglu1-mesa-dev freeglut3-dev mesa-common-dev
