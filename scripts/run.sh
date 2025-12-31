#!/bin/bash
set -e
TARGET="dev"
DEBUG="gdb -ex=run -ex=bt -ex=quit --args"
PROFILE_RECORD="perf record -g --call-graph=dwarf"
PROFILE_ANNOTATE="perf annotate --stdio > profile.txt"

run_gdb() {
  gdb \
    -ex=run \
    -ex=bt \
    -ex="set confirm off" \
    -ex=quit \
    --args "$1" "$2" "$3"
}

run_memcheck() {
  valgrind --leak-check=full $1 $2 $3
}

CMD="./build/unittests"
# // calib
# CMD="./build/unittests --gtest_filter=AprilGrid.*"
# CMD="./build/unittests --gtest_filter=AprilGridDetector.*"
# CMD="./build/unittests --gtest_filter=CalibCamera.*"
# CMD="./build/unittests --gtest_filter=CalibCameraImu.*"
# CMD="./build/unittests --gtest_filter=CalibProblem.*"
# CMD="./build/unittests --gtest_filter=CameraChain.*"
# CMD="./build/unittests --gtest_filter=LissajousTrajectory.*"
# // camera
# CMD="./build/unittests --gtest_filter=BrownConrady4.*"
# CMD="./build/unittests --gtest_filter=KannalaBrandt4.*"
# CMD="./build/unittests --gtest_filter=Pinhole.*"
# // imu
# CMD="./build/unittests --gtest_filter=ImuBuffer.*"
# // ceres
# CMD="./build/unittests --gtest_filter=CalibCameraError.*"
# CMD="./build/unittests --gtest_filter=CalibCameraImuError.*"
# CMD="./build/unittests --gtest_filter=ImuError.*"
# CMD="./build/unittests --gtest_filter=ReprojError.*"
# // core
# CMD="./build/unittests --gtest_filter=Logger.*"
# // sim
# CMD="./build/unittests --gtest_filter=SimCalib.*"
# CMD="./build/unittests --gtest_filter=SimImu.*"
# CMD="./build/unittests --gtest_filter=SimVio.*"
# // timeline
# CMD="./build/unittests --gtest_filter=Timeline.*"

tmux send-keys -t $TARGET -R C-l C-m
tmux send-keys -t $TARGET -R "\
cd $HOME/code/cartesian \
  && time make release -j \
  && ${CMD}
" C-m C-m
