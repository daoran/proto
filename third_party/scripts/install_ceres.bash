#!/bin/bash
set -e  # Exit on first error
source "config.bash"

# VERY IMPORTANT! We need to specifically tell cmake we have our own custom
# installation of eigen3 or else you'll end up with seg-faults during
# optimization
# export CMAKE_EXTRA_ARGS="-DEigen3_DIR=${PREFIX}/share/eigen3/cmake/"

# install_dependencies() {
#     apt-get update -qq
#     apt-get install -qq -y cmake \
#                            libgoogle-glog-dev \
#                            libatlas-base-dev \
#                            libeigen3-dev \
#                            libsuitesparse-dev
# }
#
# # MAIN
# install_dependencies
# install_git_repo \
#   https://github.com/ceres-solver/ceres-solver.git \
#   ceres-solver

# # Go into repo
# cd "$SRC_PATH/ceres-solver" || return

# # Prepare for build
# mkdir -p build
# cd build || return
# cmake .. \
#   -DCMAKE_BUILD_TYPE=$BUILD_TYPE \
#   -DCMAKE_INSTALL_PREFIX="$PREFIX"

# apt_install libceres-dev
