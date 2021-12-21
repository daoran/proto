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

# $ARDUINO --upload firmware/firmware.ino --port /dev/ttyUSB0
# $ARDUINO --upload firmware/firmware.ino

###############################################################################
# PYTHON
###############################################################################

# node js/proto.js
# cd python && ctags proto.py && cd -
# python3 python/proto.py
# python3 python/proto.py TestLinearAlgebra
# python3 python/proto.py TestTransform
# python3 python/proto.py TestTransform.test_quat2rot
# python3 python/proto.py TestTransform.test_rot2quat
# python3 python/proto.py TestTransform.test_rot2euler
# python3 python/proto.py TestTransform.test_quat_inv
# python3 python/proto.py TestTransform.test_quat_conj
# python3 python/proto.py TestTransform.test_quat_slerp
# python3 python/proto.py TestCV
# python3 python/proto.py TestFactors
# python3 python/proto.py TestFactors.test_pose_factor
# python3 python/proto.py TestFactors.test_ba_factor
# python3 python/proto.py TestFactors.test_vision_factor
# python3 python/proto.py TestFactors.test_calib_vision_factor
# python3 python/proto.py TestFactors.test_imu_factor_propagate
# python3 python/proto.py TestFactors.test_imu_factor
# python3 python/proto.py TestFactorGraph
python3 python/proto.py TestFactorGraph.test_factor_graph_solve_vo
# python3 python/proto.py TestFactorGraph.test_factor_graph_solve_io
# python3 python/proto.py TestFactorGraph.test_factor_graph_solve_vio
# python3 python/proto.py TestFeatureTracking
# python3 python/proto.py TestFeatureTracking.test_feature_grid_cell_index
# python3 python/proto.py TestFeatureTracking.test_feature_grid_count
# python3 python/proto.py TestFeatureTracking.test_spread_keypoints
# python3 python/proto.py TestFeatureTracking.test_grid_detect
# python3 python/proto.py TestFeatureTracking.test_optflow_track
# python3 python/proto.py TestFeatureTracker
# python3 python/proto.py TestFeatureTracker.test_detect
# python3 python/proto.py TestFeatureTracker.test_detect_overlaps
# python3 python/proto.py TestFeatureTracker.test_detect_nonoverlaps
# python3 python/proto.py TestFeatureTracker.test_detect_new
# python3 python/proto.py TestFeatureTracker.test_initialize
# python3 python/proto.py TestFeatureTracker.test_update
# python3 python/proto.py TestTracker
# python3 python/proto.py TestTracker.test_tracker_process_features
# python3 python/proto.py TestTracker.test_tracker_vision_callback
# python3 python/proto.py TestCalibration
# python3 python/proto.py TestEuroc
# python3 python/proto.py TestKitti
# python3 python/proto.py TestSimulation
# python3 python/proto.py TestSimulation.test_create_3d_features
# python3 python/proto.py TestSimulation.test_create_3d_features_perimeter
# python3 python/proto.py TestSimulation.test_sim_camera_frame
# python3 python/proto.py TestSimulation.test_sim_data
# python3 python/proto.py TestSimulation.test_sim_feature_tracker
cd python && ctags proto.py

###############################################################################
# C
###############################################################################

# make format_code
# ctags -R lib
# time make

# cd ./build/bin
# ./test_proto
# PROTO-LOGGING
# ./test_proto --target test_debug
# ./test_proto --target test_log_error
# ./test_proto --target test_log_warn
# PROTO-FILE_SYSTEM
# ./test_proto --target test_path_file_name
# ./test_proto --target test_path_file_ext
# ./test_proto --target test_path_dir_name
# ./test_proto --target test_path_join
# ./test_proto --target test_list_files
# ./test_proto --target test_list_files_free
# ./test_proto --target test_file_read
#  ./test_proto --target test_file_copy
# PROTO-DATA
# ./test_proto --target test_malloc_string
# ./test_proto --target test_dsv_rows
# ./test_proto --target test_dsv_cols
# ./test_proto --target test_dsv_fields
# ./test_proto --target test_dsv_data
# ./test_proto --target test_dsv_free
# PROTO-TIME
# ./test_proto --target test_tic
# ./test_proto --target test_toc
# ./test_proto --target test_mtoc
# ./test_proto --target test_time_now
# PROTO-MATHS
# ./test_proto --target test_min
# ./test_proto --target test_max
# ./test_proto --target test_randf
# ./test_proto --target test_deg2rad
# ./test_proto --target test_rad2deg
# ./test_proto --target test_fltcmp
# ./test_proto --target test_fltcmp2
# ./test_proto --target test_pythag
# ./test_proto --target test_lerp
# ./test_proto --target test_lerp3
# ./test_proto --target test_sinc
# ./test_proto --target test_mean
# ./test_proto --target test_median
# ./test_proto --target test_var
# ./test_proto --target test_stddev
# PROTO-LINEAR_ALGEBRA
# ./test_proto --target test_eye
# ./test_proto --target test_ones
# ./test_proto --target test_protos
# ./test_proto --target test_mat_set
# ./test_proto --target test_mat_val
# ./test_proto --target test_mat_copy
# ./test_proto --target test_mat_row_set
# ./test_proto --target test_mat_col_set
# ./test_proto --target test_mat_block_get
# ./test_proto --target test_mat_block_set
# ./test_proto --target test_mat_diag_get
# ./test_proto --target test_mat_diag_set
# ./test_proto --target test_mat_triu
# ./test_proto --target test_mat_tril
# ./test_proto --target test_mat_trace
# ./test_proto --target test_mat_transpose
# ./test_proto --target test_mat_add
# ./test_proto --target test_mat_sub
# ./test_proto --target test_mat_scale
# ./test_proto --target test_vec_add
# ./test_proto --target test_vec_sub
# ./test_proto --target test_dot
# ./test_proto --target test_skew
# ./test_proto --target test_check_jacobian
# PROTO-SVD
# ./test_proto --target test_svd
# ./test_proto --target test_svdcomp
# ./test_proto --target test_pinv
# PROTO-CHOL
# ./test_proto --target test_chol
# ./test_proto --target test_chol_lls_solve
# ./test_proto --target test_chol_lls_solve2
# ./test_proto --target test_chol_Axb
# PROTO-TIME
# PROTO-TRANSFORMS
# ./test_proto --target test_tf_set_rot
# ./test_proto --target test_tf_set_trans
# ./test_proto --target test_tf_trans
# ./test_proto --target test_tf_rot
# ./test_proto --target test_tf_quat
# ./test_proto --target test_tf_inv
# ./test_proto --target test_tf_point
# ./test_proto --target test_tf_hpoint
# ./test_proto --target test_tf_perturb_rot
# ./test_proto --target test_tf_perturb_trans
# ./test_proto --target test_quat2rot
# PROTO-POSE
# ./test_proto --target test_pose_init
# ./test_proto --target test_pose_set_get_quat
# ./test_proto --target test_pose_set_get_trans
# ./test_proto --target test_pose2tf
# ./test_proto --target test_load_poses
# PROTO-CV
# ./test_proto --target test_lie_Exp_Log
# ./test_proto --target test_linear_triangulation
# ./test_proto --target test_radtan4_distort
# ./test_proto --target test_radtan4_point_jacobian
# ./test_proto --target test_radtan4_params_jacobian
# ./test_proto --target test_equi4_distort
# ./test_proto --target test_equi4_point_jacobian
# ./test_proto --target test_equi4_params_jacobian
# ./test_proto --target test_pinhole_focal
# ./test_proto --target test_pinhole_K
# ./test_proto --target test_pinhole_project
# ./test_proto --target test_pinhole_projection_matrix
# ./test_proto --target test_pinhole_point_jacobian
# ./test_proto --target test_pinhole_params_jacobian
# ./test_proto --target test_pinhole_radtan4_project
# ./test_proto --target test_pinhole_radtan4_project_jacobian
# ./test_proto --target test_pinhole_radtan4_params_jacobian
# ./test_proto --target test_pinhole_equi4_project
# ./test_proto --target test_pinhole_equi4_project_jacobian
# ./test_proto --target test_pinhole_equi4_params_jacobian
# PROTO-SIM
# memcheck ./test_proto --target test_load_sim_features
# memcheck ./test_proto --target test_load_sim_imu_data
# memcheck ./test_proto --target test_load_sim_cam_frame
# memcheck ./test_proto --target test_load_sim_cam_data
# PROTO-SF
# ./test_proto --target test_pose_setup
# ./test_proto --target test_speed_bias_setup
# ./test_proto --target test_landmark_setup
# ./test_proto --target test_extrinsics_setup
# ./test_proto --target test_camera_setup
# ./test_proto --target test_pose_factor_setup
# ./test_proto --target test_pose_factor_eval
# ./test_proto --target test_ba_factor_setup
# ./test_proto --target test_ba_factor_eval
# ./test_proto --target test_ba_factor_ceres_eval
# ./test_proto --target test_cam_factor_setup
# ./test_proto --target test_cam_factor_eval
# ./test_proto --target test_cam_factor_ceres_eval
# ./test_proto --target test_imu_buf_setup
# ./test_proto --target test_imu_buf_add
# ./test_proto --target test_imu_buf_clear
# ./test_proto --target test_imu_buf_copy
# ./test_proto --target test_imu_buf_print
# ./test_proto --target test_ceres_solver
# ./test_proto --target test_graph_setup
# ./test_proto --target test_graph_print
# memcheck ./test_proto --target test_graph_eval

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

# ./test_imshow
# ./test_imshow2
# gdb -ex run -ex bt ./test_imshow
# ./test_template
