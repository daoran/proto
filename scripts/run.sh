set -e

export CC=/usr/bin/clang
export CXX=/usr/bin/clang++

# bash ./scripts/format_code.bash

doxygen Doxyfile

# cd octave/vision
# octave test_radtan4_distort.m
# octave test_radtan4_undistort.m
# octave sandbox.m
# octave sandbox2.m
# exit

# rm -rf build
# mkdir -p build
# cd build || return
# cmake ..
# time make -j8

# cd tests
# ./calib-aprilgrid_test
# ./calib-calib_test
# ./calib-calib_camera_test
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
# ./vision-camera-camera_geometry_test
# ./vision-camera-equi_test
# ./vision-camera-pinhole_test
# ./vision-camera-radtan_test
# ./vision-feature2d-grid_fast_test
# ./vision-feature2d-grid_good_test

# sudo make install
