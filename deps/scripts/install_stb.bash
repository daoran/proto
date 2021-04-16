#!/bin/bash
set -e  # halt on first error
source "config.bash"

clone_git_repo https://github.com/nothings/stb stb
cp -Rv $DOWNLOAD_PATH/stb/*.h $PREFIX/include
