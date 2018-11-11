#!/bin/bash
set -e  # Exit on first error
BASEDIR=$(dirname "$0")
source "$BASEDIR"/config.bash

export DOWNLOAD_PATH=$DOWNLOAD_PATH
mkdir -p "$PREFIX"
mkdir -p "$DOWNLOAD_PATH"

apt-get install -qq -y git mercurial cmake g++

echo "Installing boost ..." && "$BASEDIR"/install_boost.bash
echo "Installing yamlcpp ..." && "$BASEDIR"/install_yamlcpp.bash
echo "Installing eigen ..." && "$BASEDIR"/install_eigen.bash
echo "Installing ceres ..." && "$BASEDIR"/install_ceres.bash
echo "Installing opencv3 ..." && "$BASEDIR"/install_opencv3.bash
echo "Installing apriltags ..." && "$BASEDIR"/install_apriltags.bash
