#!/bin/bash
set -e  # Exit on first error
source "config.bash"

SCRIPTPATH="$( cd -- "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"
PREFIX="$SCRIPTPATH"
SRC_PATH="$PREFIX/src"
export DEBIAN_FRONTEND="noninteractive"
export PREFIX=$PREFIX
export SRC_PATH=$SRC_PATH

check_dir "$PREFIX/bin"
check_dir "$PREFIX/include"
check_dir "$PREFIX/lib"
check_dir "$PREFIX/log"
check_dir "$PREFIX/share"
check_dir "$PREFIX/src"

apt_update
install_base
install ros
install boost
install ceres
install eigen
install lapacke
install opencv
install apriltag
install glew
install sdl2
install suitesparse
install yamlcpp
install tiscamera
