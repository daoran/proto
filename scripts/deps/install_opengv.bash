#!/bin/bash
set -e  # halt on first error
BASEDIR=$(dirname "$0")
source "$BASEDIR/config.bash"
REPO_URL=https://github.com/laurentkneip/opengv

install_git_repo $REPO_URL opengv
