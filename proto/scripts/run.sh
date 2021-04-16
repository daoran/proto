#!/bin/sh
set -e
# ARDUINO=~/Downloads/arduino-1.8.12/arduino

debug() {
  gdb \
    -ex=run \
    -ex=bt \
    -ex="set confirm off" \
    -ex=quit \
    --args "$1" "$2" "$3"
}

# $ARDUINO --upload firmware/firmware.ino --port /dev/ttyUSB0
# $ARDUINO --upload firmware/firmware.ino

# node js/zero.js

# python3 scripts/tf_point.py

# make format_code
# ctags -R zero
make clean
time make

# doxygen
# python3 scripts/api.py
# python3 scripts/api2.py


cd ./build/bin
# ZERO-LOGGING
# ./test_zero --target test_debug
# ./test_zero --target test_log_error
# ./test_zero --target test_log_warn
# ZERO-DATA
# ./test_zero --target test_malloc_string
# ./test_zero --target test_csv_rows
# ./test_zero --target test_csv_cols
# ./test_zero --target test_csv_fields
# ./test_zero --target test_csv_data
# ZERO-LINEAR_ALGEBRA
# ./test_zero --target test_eye
# ./test_zero --target test_ones
# ./test_zero --target test_zeros
# ./test_zero --target test_mat_set
# ./test_zero --target test_mat_val
# ./test_zero --target test_mat_block_get
# ./test_zero --target test_mat_block_set
# ./test_zero --target test_mat_diag_get
# ./test_zero --target test_mat_diag_set
# ./test_zero --target test_mat_triu
# ./test_zero --target test_mat_tril
# ./test_zero --target test_mat_trace
# ./test_zero --target test_mat_transpose
# ./test_zero --target test_mat_add
# ./test_zero --target test_mat_sub
# ./test_zero --target test_mat_scale
# ./test_zero --target test_vec_add
# ./test_zero --target test_vec_sub
# ./test_zero --target test_dot
# ./test_zero --target test_skew
# ./test_zero --target test_check_jacobian
# ZERO-SVD
# ./test_zero --target test_svd
# ./test_zero --target test_svdcomp
# ./test_zero --target test_pinv
# ZERO-CHOL
# ./test_zero --target test_chol
# ./test_zero --target test_chol_lls_solve
# ./test_zero --target test_chol_lls_solve2
# ./test_zero --target test_chol_Axb
# ZERO-TIME
# ZERO-TRANSFORMS
# ./test_zero --target test_tf_set_rot
# ./test_zero --target test_tf_set_trans
# ./test_zero --target test_tf_trans
# ./test_zero --target test_tf_rot
# ./test_zero --target test_tf_quat
# ./test_zero --target test_tf_inv
# ./test_zero --target test_tf_point
# ./test_zero --target test_tf_hpoint
# ./test_zero --target test_tf_perturb_rot
# ./test_zero --target test_tf_perturb_trans
# ./test_zero --target test_quat2rot
# ZERO-POSE
# ./test_zero --target test_pose_init
# ./test_zero --target test_pose_set_get_quat
# ./test_zero --target test_pose_set_get_trans
# ./test_zero --target test_pose2tf
# ./test_zero --target test_load_poses
# ZERO-CV
# ./test_zero --target test_radtan4_distort
# ./test_zero --target test_radtan4_point_jacobian
# ./test_zero --target test_radtan4_params_jacobian
# ./test_zero --target test_equi4_distort
# ./test_zero --target test_equi4_point_jacobian
# ./test_zero --target test_equi4_params_jacobian
# ./test_zero --target test_pinhole_K
# ./test_zero --target test_pinhole_focal
# ./test_zero --target test_pinhole_project
# ./test_zero --target test_pinhole_point_jacobian
# ./test_zero --target test_pinhole_params_jacobian
# ./test_zero --target test_pinhole_radtan4_project
# ./test_zero --target test_pinhole_radtan4_project_jacobian
# ./test_zero --target test_pinhole_radtan4_params_jacobian
# ./test_zero --target test_pinhole_equi4_project
# ./test_zero --target test_pinhole_equi4_project_jacobian
# ./test_zero --target test_pinhole_equi4_params_jacobian

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
# gdb -ex run -ex bt -args ./test_gui --target test_gui_setup

# ./test_imshow
# ./test_imshow2
# gdb -ex run -ex bt ./test_imshow

# ./test_se --target test_pose_setup
# ./test_se --target test_speed_bias_setup
# ./test_se --target test_landmark_setup
# ./test_se --target test_extrinsics_setup
# ./test_se --target test_camera_setup
# ./test_se --target test_pose_factor_setup
# ./test_se --target test_pose_factor_eval
# ./test_se --target test_cam_factor_setup
# ./test_se --target test_imu_buf_setup
# ./test_se --target test_imu_buf_add
# ./test_se --target test_imu_buf_clear
# ./test_se --target test_imu_buf_copy
# ./test_se --target test_imu_buf_print
# ./test_template
