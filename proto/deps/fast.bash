#!/bin/bash
set -e
BUILD_TYPE=Release
INSTALL_PREFIX=${PWD}

# Download repo
if [ ! -d src/fast ]; then
  cd src
  git clone https://github.com/edrosten/fast-C-src fast
  cd -
fi
