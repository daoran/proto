#!/bin/bash
set -e  # Exit on first error
BASEDIR=$(dirname "$0")
source "$BASEDIR"/config.bash

export DOWNLOAD_PATH=$DOWNLOAD_PATH
mkdir -p "$PREFIX"
mkdir -p "$DOWNLOAD_PATH"

apt-get update -qqq
apt-get install -qqq -y git mercurial cmake g++

echo "Installing apriltags ..." && "$BASEDIR"/install_apriltags.bash
# echo "Installing assimp ..." && "$BASEDIR"/install_assimp.bash
echo "Installing boost ..." && "$BASEDIR"/install_boost.bash
echo "Installing ceres ..." && "$BASEDIR"/install_ceres.bash
echo "Installing eigen ..." && "$BASEDIR"/install_eigen.bash
echo "Installing geographiclib ..." && "$BASEDIR"/install_geographiclib.bash
echo "Installing glfw3 ..." && "$BASEDIR"/install_glfw3.bash
echo "Installing glm ..." && "$BASEDIR"/install_glm.bash
echo "Installing opencv3 ..." && "$BASEDIR"/install_opencv3.bash
echo "Installing opengl ..." && "$BASEDIR"/install_opengl.bash
echo "Installing yamlcpp ..." && "$BASEDIR"/install_yamlcpp.bash
