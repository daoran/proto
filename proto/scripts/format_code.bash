#!/bin/bash

# Run clang-format on source
if [ -d "zero" ]; then
  cd zero || exit
  find . -name "*.c" -print0 \
    | xargs --null -i  sh -c 'echo "clang-format -> {}";clang-format -i {};'
  find . -name "*.h" -print0 \
    | xargs --null -i  sh -c 'echo "clang-format -> {}";clang-format -i {};'
  cd ..
fi
