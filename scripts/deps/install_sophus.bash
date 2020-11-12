#!/bin/bash
set -e  # halt on first error
BASEDIR=$(dirname "$0")
source "$BASEDIR/config.bash"

install_git_repo \
  https://github.com/strasdat/Sophus \
  Sophus
