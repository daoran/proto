#!/bin/bash
set -e  # Exit on first error
source config.bash

# apt-get install -qqq libgeographic-dev
install_git_repo \
  https://git.code.sf.net/p/geographiclib/code \
  geographiclib
