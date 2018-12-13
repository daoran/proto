set -e
export CC=/usr/bin/clang
export CXX=/usr/bin/clang++

# sudo bash scripts/install_deps/run.bash
# sudo bash scripts/install_deps/install_realsense.bash
# bash ./scripts/format_code.bash
# cd scripts/api && python3 api.py

# OCTAVE
# cd octave
# ./tools/calib_data_summary
# octave notes/ba.m
# octave notes/quaternion.m
# octave notes/calibration/calibration.m
# octave notes/calibration/sandbox.m
# octave notes/frames.m
# octave notes/quaternion.m
# octave notes/intrinsics_jacobian.m && exit 0
# octave notes/equi4_point_jacobian.m
# octave notes/radtan4_params_jacobian.m && exit 0
# octave notes/radtan4_point_jacobian.m
# octave tools/plot_calib_euroc.m && exit 0
# octave tools/plot_calib_vicon.m && exit 0
# octave tools/plot_timestamps.m && exit 0
# octave tests/vision/test_radtan4_distort.m
# octave tests/vision/test_radtan4_undistort.m

# LIBRARY
mkdir -p build
cd build || return
cmake ..
make -j8
# sudo make install
# exit

# APPS
# cd apps
# ./scripts/octave/calib_data_summary
# ./calib_camera config/calib_camera.yaml
# ./calib_stereo config/calib_stereo.yaml
# ./calib_vicon_marker config/calib_vicon_marker.yaml
# ./detect_aprilgrid config/detect_aprilgrid.yaml
# ./validate_intrinsics config/validate_intrinsics.yaml
# ./validate_stereo config/validate_stereo.yaml

debug() {
  gdb \
    -ex=run \
    -ex=bt \
    -ex="set confirm off" \
    -ex=quit \
    --args "$1" "$2"
}

# TESTS
cd tests
# -- calib
# ./calib-aprilgrid_test
# ./calib-calib_camera_test
# ./calib-calib_data_test
# ./calib-calib_gimbal_test
# ./calib-calib_stereo_test
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
# ./core-tf_test
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
