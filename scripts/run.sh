set -e
export CC=/usr/bin/clang
export CXX=/usr/bin/clang++

# sudo bash scripts/install_deps/run.bash
# sudo bash scripts/install_deps/install_realsense.bash
# bash ./scripts/format_code.bash
# cd scripts/api && python3 api.py

cd octave
# octave notes/ba.m
# octave notes/quaternion.m
octave notes/calibration.m
# octave notes/frames.m
# octave tests/vision/test_radtan4_distort.m
# octave tests/vision/test_radtan4_undistort.m

# rm -rf build
# mkdir -p build
# cd build || return
# cmake ..
# time make -j8

# cd tests
# -- calib
# ./calib-aprilgrid_test
# ./calib-calib_camera_test
# ./calib-calib_data_test
# ./calib-calib_gimbal_test
# ./calib-chessboard_test

# -- control
# ./control-carrot_ctrl_test
# ./control-pid_test

# -- core
# ./core-config_test
# ./core-data_test
# ./core-file_test
# ./core-gps_test
# ./core-linalg_test
# ./core-math_test
# ./core-stats_test
# ./core-time_test

# -- driver
# ./driver-camera-camera_test

# -- dataset
# ./dataset-euroc_test
# ./dataset-kitti_test

# -- mav
# ./mav-att_ctrl_test
# ./mav-mission_test
# ./mav-pos_ctrl_test

# -- model
# ./model-mav_test
# ./model-two_wheel_test

# -- vision
# ./vision-camera-camera_geometry_test
# ./vision-camera-equi_test
# ./vision-camera-pinhole_test
# ./vision-camera-radtan_test
# ./vision-feature2d-grid_fast_test
# ./vision-feature2d-grid_good_test
# ./vision-util_test
