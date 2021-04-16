#!/bin/bash
set -e
BUILD_TYPE=Release
INSTALL_PREFIX=${PWD}

# Download repo
if [ ! -d src/glfw ]; then
  cd src
  git clone https://github.com/glfw/glfw
  cd -
fi

# Build
cd src/glfw
mkdir -p build
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=$INSTALL_PREFIX
make install
