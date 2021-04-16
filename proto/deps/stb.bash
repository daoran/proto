#!/bin/bash
set -e
BUILD_TYPE=Release
INSTALL_PREFIX=${PWD}

# Download repo
if [ ! -d src/stb ]; then
  cd src
  git clone https://github.com/nothings/stb
  cd -
fi

# Copy to include
cd src
cp -R stb/*.h ../include
