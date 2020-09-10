set -e
# export CC=/usr/bin/clang
# export CXX=/usr/bin/clang++
export CC=/usr/bin/gcc
export CXX=/usr/bin/g++
SCRIPT_PATH="`dirname \"$0\"`"
SCRIPT_PATH="`( cd \"$SCRIPT_PATH\" && pwd )`"

debug() {
  gdb \
    -ex=run \
    -ex=bt \
    -ex="set confirm off" \
    -ex=quit \
    --args "$1" "$2" "$3"
}

# tmux send-keys -t dev -R "\
#   cd ~/projects/proto;
#   make debug;
#   cd proto/build/tests;
#   ./estimation-test_factor --target test_swf_solve_vio
# " C-m
# exit

# CLEAN CATKIN
# cd ~/catkin_ws/
# catkin clean
# cd -

# DOCS
# cd docs && cd source; find . -type f -exec touch {} +; cd .. && make html && exit

# LIBRARY
# make deps
# time make debug
# time make release
# sudo make install
# time make ros
# sudo make debug_install
cd proto; time make
# exit

# roslaunch proto_ros calib_vicon_capture.launch
# mv /data/vicon_capture.bag /data/vicon-train.bag
# roslaunch proto_ros calib_vicon_capture.launch
# mv /data/vicon_capture.bag /data/vicon-test.bag
# rm -rf /data/intel_d435i/calib_vicon_data && roslaunch proto_ros calib_vicon.launch

# roslaunch proto_ros calib_vicon.launch && exit
# roslaunch proto_ros calib_vicon_validate.launch

# cd scripts/api/
# python3 api.py

# TESTS
cd proto/build/tests
# octave-cli scripts/core/plot_sim_data.m && exit

# -- CALIB
# ./test_calib

# -- CORE
# ./test_core
# ---- Data
# ./test_core --target test_csv_rows
# ./test_core --target test_csv_cols
# ./test_core --target test_csv2mat
# ./test_core --target test_mat2csv
# ---- Filesystem
# ./test_core --target test_file_exists
# ./test_core --target test_path_split
# ./test_core --target test_paths_combine
# ---- Config
# ./test_core --target test_config_constructor
# ./test_core --target test_config_parse_primitive
# ./test_core --target test_config_parse_array
# ./test_core --target test_config_parse_vector
# ./test_core --target test_config_parse_matrix
# ./test_core --target test_config_parser_full_example
# ---- Algebra
# ./test_core --target test_sign
# ./test_core --target test_linspace
# ./test_core --target test_linspace_timestamps
# ---- Linear Algebra
# ./test_core --target test_zeros
# ./test_core --target test_I
# ./test_core --target test_ones
# ./test_core --target test_hstack
# ./test_core --target test_vstack
# ./test_core --target test_dstack
# ./test_core --target test_skew
# ./test_core --target test_skewsq
# ./test_core --target test_enforce_psd
# ./test_core --target test_nullspace
# ./test_core --target test_covar_recover
# ---- Geometry
# ./test_core --target test_deg2rad_rad2deg
# ./test_core --target test_wrap180
# ./test_core --target test_wrap360
# ./test_core --target test_cross_track_error
# ./test_core --target test_point_left_right
# ./test_core --target test_closest_point
# ./test_core --target test_latlon_offset
# ./test_core --target test_latlon_diff
# ./test_core --target test_latlon_dist
# ---- Statistics
# ./test_core --target test_median
# ./test_core --target test_mvn
# ./test_core --target test_gauss_normal
# ---- Transform
# ./test_core --target test_tf_rot
# ./test_core --target test_tf_trans
# ---- Time
# ./test_core --target test_ts2sec
# ./test_core --target test_ns2sec
# ./test_core --target test_tic_toc
# ---- Network
# ./test_core --target test_tcp_server
# ./test_core --target test_tcp_client
# ./test_core --target test_tcp_server_config
# ./test_core --target test_tcp_client_config
# ./test_core --target test_tcp_server_client_loop
# ---- Interpolation
# ./test_core --target test_lerp
# ./test_core --target test_slerp
# ./test_core --target test_interp_pose
# ./test_core --target test_interp_poses
# ./test_core --target test_closest_poses
# ./test_core --target test_intersection
# ./test_core --target test_lerp_timestamps
# ./test_core --target test_lerp_data
# ./test_core --target test_lerp_data2
# ./test_core --target test_lerp_data3
# ./test_core --target test_ctraj
# ./test_core --target test_ctraj_get_pose
# ./test_core --target test_ctraj_get_velocity
# ./test_core --target test_ctraj_get_acceleration
# ./test_core --target test_ctraj_get_angular_velocity
# ./test_core --target test_sim_imu_measurement
# ---- Control
# ./test_core --target test_pid_construct
# ./test_core --target test_pid_setup
# ./test_core --target test_pid_update
# ./test_core --target test_pid_reset
# ./test_core --target test_carrot_ctrl_constructor
# ./test_core --target test_carrot_ctrl_configure
# ./test_core --target test_carrot_ctrl_closest_point
# ./test_core --target test_carrot_ctrl_carrot_point
# ./test_core --target test_carrot_ctrl_update
# ---- Measurements
# ./test_core --target test_imu_meas
# ./test_core --target test_imu_data
# ./test_core --target test_imu_data_add
# ./test_core --target test_imu_data_size
# ./test_core --target test_imu_data_last_ts
# ./test_core --target test_imu_data_clear
# ---- Model
# ./test_core --target test_two_wheel_constructor
# ./test_core --target test_two_wheel_update
# ./test_core --target test_mav_model_constructor
# ./test_core --target test_mav_model_update
# ---- Vision
# ./test_core --target test_feature_mask
# ./test_core --target test_grid_fast
# ./test_core --target benchmark_grid_fast
# ./test_core --target test_grid_good
# ./test_core --target test_radtan_distort_point
# ./test_core --target test_radtan_undistort_point
# ./test_core --target test_equi_distort_point
# ./test_core --target test_equi_undistort_point
# ./test_core --target test_pinhole
# ./test_core --target test_pinhole_K
# ./test_core --target test_pinhole_focal
# ./test_core --target test_pinhole_project
# ---- Simulation
# ./test_core --target test_sim_circle_trajectory

# -- dataset
# ./test_euroc
# ./test_kitti

# -- estimation
# ./test_dense
# ./test_factor
# ./test_factor --target test_pose_factor_jacobians
# ./test_factor --target test_extrinsic_factor_jacobians
# ./test_factor --target test_speed_bias_factor_jacobians
# ./test_factor --target test_camera_params_factor_jacobians
# ./test_factor --target test_ba_factor_jacobians
# ./test_factor --target test_calib_mono_factor_jacobians
# ./test_factor --target test_cam_factor_jacobians
# ./test_factor --target test_imu_factor_jacobians
# ./test_factor --target test_imu_propagate
# ./test_factor --target test_graph
# ./test_factor --target test_graph_add_pose
# ./test_factor --target test_graph_add_landmark
# ./test_factor --target test_graph_add_camera
# ./test_factor --target test_graph_add_speed_bias
# ./test_factor --target test_graph_add_pose_factor
# ./test_factor --target test_graph_add_ba_factor
# ./test_factor --target test_graph_add_cam_factor
# ./test_factor --target test_graph_add_imu_factor
# ./test_factor --target test_graph_rm_param
# ./test_factor --target test_graph_rm_factor
# ./test_factor --target test_graph_eval
# ./test_factor --target test_graph_get_state
# ./test_factor --target test_graph_set_state
# ./test_factor --target test_graph_solve_ba
# ./test_factor --target test_swf_add_imu
# ./test_factor --target test_swf_add_camera
# ./test_factor --target test_swf_add_extrinsics
# ./test_factor --target test_swf_add_feature
# ./test_factor --target test_swf_add_pose
# ./test_factor --target test_swf_add_speed_bias
# ./test_factor --target test_swf_add_pose_prior
# ./test_factor --target test_swf_add_ba_factor
# ./test_factor --target test_swf_add_imu_factor
# ./test_factor --target test_swf_add_cam_factor
# ./test_factor --target test_swf_add_cam_factor
# debug ./test_factor --target test_swf_pre_marginalize
# ./test_factor --target test_swf_marginalize
# ./test_factor --target test_swf_solve_vo
# ./test_factor --target test_swf_solve_vio

# ./test_frontend
