set -e

export LD_LIBRARY_PATH=/usr/local/src/mexopencv/lib

make

# sudo bash ./scripts/install_mexopencv.bash
# ./scripts/run_tests.bash && exit 0
# octave notes/ba/ba.m && exit 0
# octave notes/ba/ba_nbv.m && exit 0
# octave notes/calib/calib.m && exit 0
# octave notes/camera/equi4_point_jacobian.m && exit 0
# octave notes/camera/intrinsics_jacobian.m && exit 0
# octave notes/camera/radtan4_params_jacobian.m && exit 0
# octave notes/camera/radtan4_point_jacobian.m && exit 0
# octave notes/imu_preintegration/ipm.m && exit 0
# octave notes/imu_preintegration/ipm_jacobian.m && exit 0
# octave notes/visync/main.m && exit 0
# octave notes/back_substitution.m && exit 0
# octave notes/bspline.m && exit 0
# octave notes/convolution.m && exit 0
# octave notes/error_ellipse.m && exit 0
# octave notes/error_propagation.m && exit 0
# octave notes/euroc_frames.m && exit 0
# octave notes/gauss_newton.m && exit 0
# octave notes/harris_corner.m && exit 0
# octave notes/illum_invar.m && exit 0
# octave notes/jacobians.m && exit 0
# octave notes/linear_triangulation.m && exit 0
# octave notes/quaternion.m && exit 0
octave notes/nbt.m && exit 0

# octave tools/calib_data_summary.m && exit 0
# octave tools/plot_aprilgrid_data.m && exit 0
# octave tools/plot_calib_euroc.m && exit 0
# octave tools/plot_timestamps.m && exit 0
# octave tools/plot_ubx_nav_hpposllh.m

# octave tests/calib/test_calib_generate_poses.m && exit 0
# octave tests/calib/test_calib_generate_random_poses.m && exit 0
# octave tests/calib/test_calib_target_init.m && exit 0
# octave tests/calib/test_calib_target_draw.m && exit 0
# octave tests/calib/test_calib_sim.m && exit 0

# octave tests/control/test_pid_init.m && exit 0
# octave tests/control/test_pid_update.m && exit 0

# octave tests/core/config/test_config.m && exit 0
# octave tests/core/os/test_join_paths.m && exit 0
# octave tests/core/os/test_list_dir.m && exit 0
# octave tests/core/quat/test_quat_mul.m && exit 0
# octave tests/core/quat/test_quat_norm.m && exit 0
# octave tests/core/quat/test_quat_normalize.m && exit 0
# octave tests/core/quat/test_quat_omega.m && exit 0
# octave tests/core/tf/test_dehomogeneous.m && exit 0
# octave tests/core/tf/test_euler2quat.m && exit 0
# octave tests/core/tf/test_euler321.m && exit 0
# octave tests/core/tf/test_homogeneous.m && exit 0
# octave tests/core/tf/test_perturb_rot.m && exit 0
# octave tests/core/tf/test_perturb_trans.m && exit 0
# octave tests/core/tf/test_quat2euler.m && exit 0
# octave tests/core/tf/test_rotx.m && exit 0
# octave tests/core/tf/test_roty.m && exit 0
# octave tests/core/tf/test_rotz.m && exit 0
# octave tests/core/tf/test_tf.m && exit 0
# octave tests/core/tf/test_tf_decompose.m && exit 0
# octave tests/core/tf/test_tf_inv.m && exit 0
# octave tests/core/tf/test_tf_trans.m && exit 0
# octave tests/core/test_bezier_cubic.m && exit 0
# octave tests/core/test_check_jacobian.m && exit 0
# octave tests/core/test_isapprox.m && exit 0
# octave tests/core/test_normalize.m && exit 0
# octave tests/core/test_skew.m && exit 0

# octave tests/dataset/test_load_aprilgrid.m && exit 0
# octave tests/dataset/test_load_euroc.m && exit 0
# octave tests/dataset/test_load_kitti.m && exit 0

# octave tests/model/test_bicycle_init.m && exit 0
# octave tests/model/test_bicycle_update.m && exit 0

# octave tests/plot/test_draw_camera.m && exit 0
# octave tests/plot/test_draw_frame.m && exit 0
# octave tests/plot/test_draw_points.m && exit 0

# octave tests/vision/test_camera_init.m && exit 0
# octave tests/vision/test_camera_measurements.m && exit 0
# octave tests/vision/test_chessboard_create.m && exit 0
# octave tests/vision/test_equi4_distort.m && exit 0
# octave tests/vision/test_equi4_param_jacobian.m && exit 0
# octave tests/vision/test_equi4_point_jacobian.m && exit 0
# octave tests/vision/test_equi4_undistort.m && exit 0
# octave tests/vision/test_focal_length.m && exit 0
# octave tests/vision/test_pinhole_K.m && exit 0
# octave tests/vision/test_radtan4_distort.m && exit 0
# octave tests/vision/test_radtan4_param_jacobian.m && exit 0
# octave tests/vision/test_radtan4_point_jacobian.m && exit 0
# octave tests/vision/test_radtan4_undistort.m && exit 0
# octave tests/vision/test_trajectory_plot.m && exit 0
# octave tests/vision/test_trajectory_simulate.m && exit 0
# octave tests/vision/test_webcam.m && exit 0
