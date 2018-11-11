set -e

export CC=/usr/bin/clang
export CXX=/usr/bin/clang++

# bash ./scripts/format_code.bash

# doxygen Doxyfile
# cd scripts/api && python3 api.py

# cd octave/vision
# octave sandbox.m
# octave sandbox2.m
# exit

# cd octave
# octave notes/ba.m
# octave notes/point_jacobian.m
# octave tests/vision/test_radtan4_distort.m
# octave tests/vision/test_radtan4_undistort.m

# rm -rf build
mkdir -p build
cd build || return
cmake ..
time make -j8

cd tests
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
# ./dataset-euroc_test
# ./dataset-kitti_test
 ./mav-mission_test
# ./vision-camera-camera_geometry_test
# ./vision-camera-equi_test
# ./vision-camera-pinhole_test
# ./vision-camera-radtan_test
# ./vision-feature2d-grid_fast_test
# ./vision-feature2d-grid_good_test

# sudo make install
