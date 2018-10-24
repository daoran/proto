#!/bin/bash
set -e  # Exit on first error
BASEDIR=$(dirname "$0")
source $BASEDIR/config.bash

# MAIN
install_git_repo \
  https://github.com/google/googletest \
  googletest
