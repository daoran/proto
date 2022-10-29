#!/bin/bash
set -e  # halt on first error
source "config.bash"

clone_git_repo https://github.com/edrosten/fast-C-src fast
