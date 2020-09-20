#!/bin/bash
set -e  # Exit on first error
trap "exit;" SIGINT SIGTERM  # Signal handler

run_test() {
  echo -ne "\e[34mTEST\e[0m [$1] "

  if octave --eval "source('$1')" > /dev/null 2>&1; then
    echo -ne '\e[32m'
    echo -n 'PASSED!'
    echo -ne '\e[0m\n'
  else
    echo -ne '\e[31m'
    echo -n 'FAILED!'
    echo -ne '\e[0m\n'
  fi
}

TESTS=$(find tests -name '*.m')
for TEST in $TESTS; do
  run_test "$TEST";
done
