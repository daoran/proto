#!/bin/bash
set -e
BUILD_TYPE=Release
INSTALL_PREFIX=${PWD}

# Install Deps
sudo apt-get install libgmp3-dev -y -qq
sudo apt-get install libmpfr-dev libmpfr-doc -y -qq

# Download repo
if [ ! -d src/SuiteSparse ]; then
  cd src
  git clone https://github.com/DrTimothyAldenDavis/SuiteSparse
  cd -
fi

# Build
cd src/SuiteSparse
make library
make install INSTALL=$INSTALL_PREFIX
