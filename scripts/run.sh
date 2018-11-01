set -e

export CC=/usr/bin/clang
export CXX=/usr/bin/clang++

# bash ./scripts/format_code.bash

# rm -rf build
mkdir -p build
cd build || return
cmake ..
time make -j8

cd tests
# ./calib-aprilgrid_test
./calib-calib_test
# ./control-mission_test
# ./control-pid_test
# ./core-config_test
# ./core-data_test
# ./core-file_test
# ./core-gps_test
# ./core-linalg_test
# ./core-math_test
# ./core-stats_test
# ./core-time_test
# ./driver-camera-camera_test

# sudo make install
