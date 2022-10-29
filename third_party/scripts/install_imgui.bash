#!/bin/bash
set -e  # halt on first error
source "config.bash"

REPO_NAME=imgui
REPO_URL=https://github.com/ocornut/imgui
DOWNLOAD_PATH=${PREFIX}/src

# Clone repo
mkdir -p $DOWNLOAD_PATH
cd $DOWNLOAD_PATH || return
if [ ! -d $REPO_NAME ]; then
  git clone $REPO_URL $REPO_NAME
fi
