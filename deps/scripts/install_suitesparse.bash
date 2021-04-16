#!/bin/bash
set -e
source "config.bash"

# Install Deps
sudo apt-get install libgmp3-dev -y -qq
sudo apt-get install libmpfr-dev libmpfr-doc -y -qq

# Download repo
clone_git_repo \
  https://github.com/DrTimothyAldenDavis/SuiteSparse \
  SuiteSparse

# Build
cd $DOWNLOAD_PATH/SuiteSparse
make library
make install INSTALL=$INSTALL_PREFIX
