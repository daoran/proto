#!/bin/bash
set -e  # Exit on first error
BASEDIR=$(dirname "$0")
source $BASEDIR/config.bash

# apt-get install -qqq libgeographic-dev
install_git_repo \
  https://git.code.sf.net/p/geographiclib/code \
  geographiclib
