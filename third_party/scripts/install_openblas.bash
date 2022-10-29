#!/bin/bash
set -e
source "config.bash"

clone_git_repo https://github.com/xianyi/OpenBLAS OpenBLAS
cd $DOWNLOAD_PATH/OpenBLAS
make
