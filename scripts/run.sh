#!/bin/sh
set -e

debug() {
  gdb \
    -ex=run \
    -ex=bt \
    -ex="set confirm off" \
    -ex=quit \
    --args "$1" "$2" "$3"
}

memcheck() {
  valgrind --leak-check=full $1 $2 $3
}

###############################################################################
# PYTHON
###############################################################################

# cd python && ctags proto.py && cd -
# python3 proto/python/proto.py
# python3 proto/python/proto.py TestLinearAlgebra
# python3 proto/python/proto.py TestTransform
# python3 proto/python/proto.py TestTransform.test_quat2rot
# python3 proto/python/proto.py TestTransform.test_rot2quat
# python3 proto/python/proto.py TestTransform.test_rot2euler
# python3 proto/python/proto.py TestTransform.test_quat_inv
# python3 proto/python/proto.py TestTransform.test_quat_conj
# python3 proto/python/proto.py TestTransform.test_quat_slerp
# python3 proto/python/proto.py TestCV
# python3 proto/python/proto.py TestFactors
# python3 proto/python/proto.py TestFactors.test_pose_factor
# python3 proto/python/proto.py TestFactors.test_ba_factor
# python3 proto/python/proto.py TestFactors.test_vision_factor
# python3 proto/python/proto.py TestFactors.test_calib_vision_factor
# python3 proto/python/proto.py TestFactors.test_imu_factor_propagate
# python3 proto/python/proto.py TestFactors.test_imu_factor
# python3 proto/python/proto.py TestFactorGraph
# python3 proto/python/proto.py TestFactorGraph.test_factor_graph_solve_vo
# python3 proto/python/proto.py TestFactorGraph.test_factor_graph_solve_io
# python3 proto/python/proto.py TestFactorGraph.test_factor_graph_solve_vio
# python3 proto/python/proto.py TestFeatureTracking
# python3 proto/python/proto.py TestFeatureTracking.test_feature_grid_cell_index
# python3 proto/python/proto.py TestFeatureTracking.test_feature_grid_count
# python3 proto/python/proto.py TestFeatureTracking.test_spread_keypoints
# python3 proto/python/proto.py TestFeatureTracking.test_grid_detect
# python3 proto/python/proto.py TestFeatureTracking.test_optflow_track
# python3 proto/python/proto.py TestFeatureTracker
# python3 proto/python/proto.py TestFeatureTracker.test_detect
# python3 proto/python/proto.py TestFeatureTracker.test_detect_overlaps
# python3 proto/python/proto.py TestFeatureTracker.test_detect_nonoverlaps
# python3 proto/python/proto.py TestFeatureTracker.test_detect_new
# python3 proto/python/proto.py TestFeatureTracker.test_initialize
# python3 proto/python/proto.py TestFeatureTracker.test_update
# python3 proto/python/proto.py TestTracker
# python3 proto/python/proto.py TestTracker.test_tracker_process_features
# python3 proto/python/proto.py TestTracker.test_tracker_vision_callback
# python3 proto/python/proto.py TestCalibration
# python3 proto/python/proto.py TestCalibration.test_aprilgrid
# python3 proto/python/proto.py TestCalibration.test_calibrator
# python3 proto/python/proto.py TestEuroc
# python3 proto/python/proto.py TestKitti
# python3 proto/python/proto.py TestSimulation
# python3 proto/python/proto.py TestSimulation.test_create_3d_features
# python3 proto/python/proto.py TestSimulation.test_create_3d_features_perimeter
# python3 proto/python/proto.py TestSimulation.test_sim_camera_frame
# python3 proto/python/proto.py TestSimulation.test_sim_data
# python3 proto/python/proto.py TestSimulation.test_sim_feature_tracker
# python3 proto/python/proto.py TestViz.test_multiplot
# python3 proto/python/proto.py TestViz.test_server

###############################################################################
# C
###############################################################################

# make format_code
# ctags -R lib
time make clean
time make build

run_test() {
  cd ./proto/build/bin;
  ./test_proto --target="$1"
}

# ./test_proto
# PROTO-LOGGING
# run_test test_debug
run_test test_log_error
# run_test test_log_warn
# PROTO-FILE_SYSTEM
# run_test test_path_file_name
# run_test test_path_file_ext
# run_test test_path_dir_name
# run_test test_path_join
# run_test test_list_files
# run_test test_list_files_free
# run_test test_file_read
#  run_test test_file_copy
# PROTO-DATA
# run_test test_malloc_string
# run_test test_dsv_rows
# run_test test_dsv_cols
# run_test test_dsv_fields
# run_test test_dsv_data
# run_test test_dsv_free
# PROTO-TIME
# run_test test_tic
# run_test test_toc
# run_test test_mtoc
# run_test test_time_now
# PROTO-MATHS
# run_test test_min
# run_test test_max
# run_test test_randf
# run_test test_deg2rad
# run_test test_rad2deg
# run_test test_fltcmp
# run_test test_fltcmp2
# run_test test_pythag
# run_test test_lerp
# run_test test_lerp3
# run_test test_sinc
# run_test test_mean
# run_test test_median
# run_test test_var
# run_test test_stddev
# PROTO-LINEAR_ALGEBRA
# run_test test_eye
# run_test test_ones
# run_test test_protos
# run_test test_mat_set
# run_test test_mat_val
# run_test test_mat_copy
# run_test test_mat_row_set
# run_test test_mat_col_set
# run_test test_mat_block_get
# run_test test_mat_block_set
# run_test test_mat_diag_get
# run_test test_mat_diag_set
# run_test test_mat_triu
# run_test test_mat_tril
# run_test test_mat_trace
# run_test test_mat_transpose
# run_test test_mat_add
# run_test test_mat_sub
# run_test test_mat_scale
# run_test test_vec_add
# run_test test_vec_sub
# run_test test_dot
# run_test test_skew
# run_test test_check_jacobian
# PROTO-SVD
# run_test test_svd
# run_test test_svdcomp
# run_test test_pinv
# PROTO-CHOL
# run_test test_chol
# run_test test_chol_lls_solve
# run_test test_chol_lls_solve2
# run_test test_chol_Axb
# PROTO-TIME
# PROTO-TRANSFORMS
# run_test test_tf_set_rot
# run_test test_tf_set_trans
# run_test test_tf_trans
# run_test test_tf_rot
# run_test test_tf_quat
# run_test test_tf_inv
# run_test test_tf_point
# run_test test_tf_hpoint
# run_test test_tf_perturb_rot
# run_test test_tf_perturb_trans
# run_test test_quat2rot
# PROTO-POSE
# run_test test_pose_init
# run_test test_pose_set_get_quat
# run_test test_pose_set_get_trans
# run_test test_pose2tf
# run_test test_load_poses
# PROTO-CV
# run_test test_lie_Exp_Log
# run_test test_linear_triangulation
# run_test test_radtan4_distort
# run_test test_radtan4_point_jacobian
# run_test test_radtan4_params_jacobian
# run_test test_equi4_distort
# run_test test_equi4_point_jacobian
# run_test test_equi4_params_jacobian
# run_test test_pinhole_focal
# run_test test_pinhole_K
# run_test test_pinhole_project
# run_test test_pinhole_projection_matrix
# run_test test_pinhole_point_jacobian
# run_test test_pinhole_params_jacobian
# run_test test_pinhole_radtan4_project
# run_test test_pinhole_radtan4_project_jacobian
# run_test test_pinhole_radtan4_params_jacobian
# run_test test_pinhole_equi4_project
# run_test test_pinhole_equi4_project_jacobian
# run_test test_pinhole_equi4_params_jacobian
# PROTO-SIM
# memcheck run_test test_load_sim_features
# memcheck run_test test_load_sim_imu_data
# memcheck run_test test_load_sim_cam_frame
# memcheck run_test test_load_sim_cam_data
# PROTO-SF
# run_test test_pose_setup
# run_test test_speed_bias_setup
# run_test test_landmark_setup
# run_test test_extrinsics_setup
# run_test test_camera_setup
# run_test test_pose_factor_setup
# run_test test_pose_factor_eval
# run_test test_ba_factor_setup
# run_test test_ba_factor_eval
# run_test test_ba_factor_ceres_eval
# run_test test_cam_factor_setup
# run_test test_cam_factor_eval
# run_test test_cam_factor_ceres_eval
# run_test test_imu_buf_setup
# run_test test_imu_buf_add
# run_test test_imu_buf_clear
# run_test test_imu_buf_copy
# run_test test_imu_buf_print
# run_test test_ceres_solver
# run_test test_graph_setup
# run_test test_graph_print
# memcheck run_test test_graph_eval

# valgrind --leak-check=full ./test_traj_eval
# time ./test_traj_eval

# ./test_gui
# ./test_gui --target test_gl_zeros
# ./test_gui --target test_gl_ones
# ./test_gui --target test_gl_eye
# ./test_gui --target test_gl_matf_set
# ./test_gui --target test_gl_matf_val
# ./test_gui --target test_gl_transpose
# ./test_gui --target test_gl_equals
# ./test_gui --target test_gl_vec3_cross
# ./test_gui --target test_gl_dot
# ./test_gui --target test_gl_norm
# ./test_gui --target test_gl_normalize
# ./test_gui --target test_gl_perspective
# ./test_gui --target test_gl_lookat
# ./test_gui --target test_shader_compile
# ./test_gui --target test_shader_link
# ./test_gui --target test_gl_prog_setup
# ./test_gui --target test_gl_camera_setup
# ./test_gui --target test_gui
# ./test_gui --target test_imshow
# gdb -ex run -ex bt -args ./test_gui --target test_gui_setup

###############################################################################
# ARDUINO
###############################################################################

# $ARDUINO --upload firmware/firmware.ino --port /dev/ttyUSB0
# $ARDUINO --upload firmware/firmware.ino
