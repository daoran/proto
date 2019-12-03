#!/bin/sh
set -e

find . -name "*.cpp" -print0 | xargs --null -i  sh -c 'echo "clang-format -> {}";clang-format -style=file -i {};'
find . -name "*.hpp" -print0 | xargs --null -i  sh -c 'echo "clang-format -> {}";clang-format -style=file -i {};'
