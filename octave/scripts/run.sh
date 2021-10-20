set -e

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/src/mexopencv/lib
ctags -R .

# make
# make tests

# sudo bash ./scripts/install_mexopencv.bash
# octave notes/covar_recover.m && exit 0
# octave notes/plot_orbslam3_data.m && exit 0
# octave notes/schurs.m && exit 0
# octave notes/aabm.m && exit 0
# octave notes/ba/ba.m && exit 0
# octave notes/ba/ba_nbv.m && exit 0
# octave notes/calib/calib.m && exit 0
# octave notes/stereo_calib/stereo_calib.m && exit 0
# octave notes/camera/equi4_point_jacobian.m && exit 0
# octave notes/camera/intrinsics_jacobian.m && exit 0
# octave notes/camera/radtan4_params_jacobian.m && exit 0
# octave notes/camera/radtan4_point_jacobian.m && exit 0
# octave notes/traj_eval/traj_eval.m
# octave notes/imu/imu.m && exit 0
# octave notes/imu/ipm_jacobian.m && exit 0
# octave notes/visync/main.m && exit 0
# octave notes/back_substitution.m && exit 0
# octave notes/bspline.m && exit 0
# octave notes/circle_sim.m && exit 0
# octave notes/convolution.m && exit 0
# octave notes/error_ellipse.m && exit 0
# octave notes/error_propagation.m && exit 0
# octave notes/euler321_deriv.m && exit 0
# octave notes/euroc_frames.m && exit 0
# octave notes/fiducial_xy_jac.m
# octave notes/fpv_frames.m && exit 0
# octave notes/frames_sandbox.m && exit 0
# octave notes/gauss_newton.m && exit 0
# octave notes/harris_corner.m && exit 0
# octave notes/illum_invar.m && exit 0
# octave notes/inv_depth_param.m && exit 0
octave notes/inv_depth_param_jacobian.m && exit 0
# octave notes/jacobians.m && exit 0
# octave notes/least_squares.m && exit 0
# octave notes/linear_triangulation.m && exit 0
# octave notes/quaternion.m && exit 0
# octave notes/mav.m && exit 0
# octave notes/msckf.m && exit 0
# octave notes/nbt.m && exit 0
# octave notes/nbt_trajs.m && exit 0
# octave notes/rand_inv_matrix.m && exit 0
# octave notes/stereo_pose_error.m && exit 0
# octave notes/vicon_frames.m && exit 0
# octave notes/vicon_f450.m && exit 0
# octave notes/vicon_ucl.m && exit 0
# octave notes/mocap_debug.m && exit 0
# octave notes/dpose.m && exit 0
# octave notes/drot.m && exit 0

# octave tools/calib_data_summary.m && exit 0
# octave tools/plot_aprilgrid_data.m && exit 0
# octave tools/plot_calib_euroc.m && exit 0
# octave tools/plot_timestamps.m && exit 0
# octave tools/plot_ubx_nav_hpposllh.m

# octave tests/calib/test_aprilgrid_grid_index.m && exit 0
# octave tests/calib/test_aprilgrid_init.m && exit 0
# octave tests/calib/test_calib_generate_poses.m && exit 0
# octave tests/calib/test_calib_generate_random_poses.m && exit 0
# octave tests/calib/test_calib_target_init.m && exit 0
# octave tests/calib/test_calib_target_draw.m && exit 0
# octave tests/calib/test_calib_sim.m && exit 0

# octave tests/control/test_pid_init.m && exit 0
# octave tests/control/test_pid_update.m && exit 0

# octave tests/core/config/test_config.m && exit 0
# octave tests/core/ctraj/test_ctraj_init.m && exit 0
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

# octave tests/cv/test_camera_measurements.m && exit 0
# octave tests/cv/test_chessboard_create.m && exit 0
# octave tests/cv/test_equi4_distort.m && exit 0
# octave tests/cv/test_equi4_param_jacobian.m && exit 0
# octave tests/cv/test_equi4_point_jacobian.m && exit 0
# octave tests/cv/test_equi4_undistort.m && exit 0
# octave tests/cv/test_focal_length.m && exit 0
# octave tests/cv/test_idp_param.m && exit 0
# octave tests/cv/test_lookat.m && exit 0
# octave tests/cv/test_pinhole_K.m && exit 0
# octave tests/cv/test_point2bearing.m && exit 0
# octave tests/cv/test_pinhole_params_jacobian.m && exit 0
# octave tests/cv/test_pinhole_point_jacobian.m && exit 0
# octave tests/cv/test_pinhole_project.m && exit 0
# octave tests/cv/test_radtan4_distort.m && exit 0
# octave tests/cv/test_radtan4_param_jacobian.m && exit 0
# octave tests/cv/test_radtan4_point_jacobian.m && exit 0
# octave tests/cv/test_radtan4_undistort.m && exit 0
# octave tests/cv/test_webcam.m && exit 0

# octave tests/dataset/test_load_aprilgrid.m && exit 0
# octave tests/dataset/test_load_euroc.m && exit 0
# octave tests/dataset/test_load_kitti.m && exit 0

# octave tests/model/test_bicycle_init.m && exit 0
# octave tests/model/test_bicycle_update.m && exit 0

# octave tests/plot/test_draw_camera.m && exit 0
# octave tests/plot/test_draw_frame.m && exit 0
# octave tests/plot/test_draw_points.m && exit 0

# octave tests/se/factor/test_ba_factor_init.m && exit 0
# octave tests/se/factor/test_ba_factor_eval.m && exit 0
# octave tests/se/factor/test_cam_factor_eval.m && exit 0
# octave tests/se/param/test_camera_init.m && exit 0
# octave tests/se/param/test_landmark_init.m && exit 0
# octave tests/se/param/test_pose_init.m && exit 0
# octave tests/se/param/test_sb_init.m && exit 0
# octave tests/se/test_graph_add_factor.m && exit 0
# octave tests/se/test_graph_add_param.m && exit 0
# octave tests/se/test_graph_eval.m && exit 0
# octave tests/se/test_graph_init.m && exit 0

# octave tests/sim/test_trajectory_plot.m && exit 0
# octave tests/sim/test_trajectory_simulate.m && exit 0
