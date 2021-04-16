#!/bin/bash
set -e
BUILD_TYPE=Release
INSTALL_PREFIX=${PWD}

# Download repo
if [ ! -d src/OpenBLAS ]; then
  cd src
	git clone https://github.com/xianyi/OpenBLAS
	cd -
fi

# Build
cd src/OpenBLAS
make
