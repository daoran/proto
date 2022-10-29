#!/bin/bash
set -e  # halt on first error
source "config.bash"
REPO_URL=https://github.com/AprilRobotics/apriltag-imgs

# Clone repo
mkdir -p "$DOWNLOAD_PATH"
cd "$DOWNLOAD_PATH" || return
if [ ! -d "$2" ]; then
  git clone "$REPO_URL"
fi
