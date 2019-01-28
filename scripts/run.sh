set -e
export CC=/usr/bin/clang
export CXX=/usr/bin/clang++

debug() {
  gdb \
    -ex=run \
    -ex=bt \
    -ex="set confirm off" \
    -ex=quit \
    --args "$1" "$2"
}

# sudo bash scripts/install_deps/run.bash
# sudo bash scripts/install_deps/install_realsense.bash
# bash ./scripts/format_code.bash
# cd scripts/api && python3 api.py

# OCTAVE
# cd octave
# export LD_LIBRARY_PATH=/usr/local/src/mexopencv/lib
# sudo bash ./install_mexopencv.bash
# ./run_tests.bash
# ./tools/calib_data_summary
# octave notes/ba.m && exit 0
# octave notes/quaternion.m && exit 0
# octave notes/calibration/calibration.m && exit 0
# octave notes/calibration/sandbox.m && exit 0
# octave notes/frames.m && exit 0
# octave notes/quaternion.m && exit 0
# octave notes/intrinsics_jacobian.m && exit 0
# octave notes/equi4_point_jacobian.m && exit 0
# octave notes/radtan4_params_jacobian.m && exit 0
# octave notes/radtan4_point_jacobian.m && exit 0
# octave tools/plot_calib_euroc.m && exit 0
# octave tools/plot_calib_vicon.m && exit 0
# octave tools/plot_timestamps.m && exit 0
# octave tools/plot_marker_poses.m && exit 0

# octave tests/control/test_pid_init.m && exit 0
# octave tests/control/test_pid_update.m && exit 0
# octave tests/model/test_bicycle_init.m && exit 0
# octave tests/model/test_bicycle_update.m && exit 0
# octave tests/core/config/test_config.m && exit 0
# octave tests/core/os/test_join_paths.m && exit 0
# octave tests/core/os/test_list_dir.m && exit 0
# octave tests/core/transform/test_dehomogeneous.m && exit 0
# octave tests/core/transform/test_euler2quat.m && exit 0
# octave tests/core/transform/test_euler321.m && exit 0
# octave tests/core/transform/test_homogeneous.m && exit 0
# octave tests/core/transform/test_perturb_rot.m && exit 0
# octave tests/core/transform/test_perturb_trans.m && exit 0
# octave tests/core/transform/test_quat2euler.m && exit 0
# octave tests/core/transform/test_rotx.m && exit 0
# octave tests/core/transform/test_roty.m && exit 0
# octave tests/core/transform/test_rotz.m && exit 0
# octave tests/core/transform/test_tf.m && exit 0
# octave tests/core/transform/test_tf_decompose.m && exit 0
# octave tests/core/transform/test_tf_inv.m && exit 0
# octave tests/core/transform/test_tf_trans.m && exit 0
# octave tests/core/quaternion/test_quat_mul.m && exit 0
# octave tests/core/quaternion/test_quat_norm.m && exit 0
# octave tests/core/quaternion/test_quat_normalize.m && exit 0
# octave tests/core/quaternion/test_quat_omega.m && exit 0
# octave tests/core/test_bezier_cubic.m && exit 0
# octave tests/core/test_check_jacobian.m && exit 0
# octave tests/core/test_isapprox.m && exit 0
# octave tests/core/test_normalize.m && exit 0
# octave tests/core/test_skew.m && exit 0

# cd prototype/dataset
# make clean
# make all
# exit 0
# octave tests/dataset/test_load_aprilgrid.m && exit 0
# octave tests/dataset/test_load_euroc.m && exit 0

# octave tests/rovio/test_feature_tracker_create.m && exit 0
# octave tests/rovio/test_feature_tracker_update.m && exit 0
# octave tests/rovio/test_imu_create.m && exit 0
# octave tests/rovio/test_imu_propagate.m && exit 0


# octave tests/plot/test_draw_camera.m && exit 0
# octave tests/plot/test_draw_chessboard.m && exit 0
# octave tests/plot/test_draw_frame.m && exit 0
# octave tests/plot/test_draw_points.m && exit 0

# octave tests/vision/test_camera_create.m && exit 0
# octave tests/vision/test_camera_measurements.m && exit 0
# octave tests/vision/test_chessboard_create.m && exit 0
# octave tests/vision/test_equi4_undistort.m && exit 0
# octave tests/vision/test_focal_length.m && exit 0
# octave tests/vision/test_pinhole_K.m && exit 0
# octave tests/vision/test_radtan4_distort.m && exit 0
# octave tests/vision/test_radtan4_undistort.m && exit 0
# octave tests/vision/test_trajectory_plot.m && exit 0
# octave tests/vision/test_trajectory_simulate.m && exit 0
# octave tests/vision/test_webcam.m && exit 0

# LIBRARY
# mkdir -p build
cd build || return
cmake ..
make -j8
sudo make install
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

# TESTS
# cd tests
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
