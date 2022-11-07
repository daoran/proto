#!/bin/bash
set -e
source "config.bash"

cd "$SRC_PATH/tiscamera"
./scripts/dependency-manager install
mkdir -p build
cd build || return
cmake \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$PREFIX" \
  -DTCAM_BUILD_ARAVIS=ON ..
make
