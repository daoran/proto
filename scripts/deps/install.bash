#!/bin/bash
set -e  # Exit on first error
BASEDIR=$(dirname "$0")
source "$BASEDIR"/config.bash

export DEBIAN_FRONTEND="noninteractive"
export DOWNLOAD_PATH=$DOWNLOAD_PATH
mkdir -p "$PREFIX"
mkdir -p "$DOWNLOAD_PATH"

install() {
  echo "Installing $1 ..." && "$BASEDIR"/install_"$1".bash > /dev/null
}

install_base() {
  apt_install dialog apt-utils git mercurial cmake g++ clang
}

apt_update
install_base
install ros
install assimp
install boost
install ceres
install eigen
install geographiclib
install glad
install glfw3
install glm
install imgui
install opencv3
install apriltags
install opengl
install opengv
install octomap
install yamlcpp
