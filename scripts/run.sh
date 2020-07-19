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
time make debug
# time make release
# sudo make install
# time make ros
# sudo make debug_install
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

# -- CORE
# ./core-test_core
# ---- Data
# ./core-test_core --target test_csv_rows
# ./core-test_core --target test_csv_cols
# ./core-test_core --target test_csv2mat
# ./core-test_core --target test_mat2csv
# ---- Filesystem
# ./core-test_core --target test_file_exists
# ./core-test_core --target test_path_split
# ./core-test_core --target test_paths_combine
# ---- Config
# ./core-test_core --target test_config_constructor
# ./core-test_core --target test_config_parse_primitive
# ./core-test_core --target test_config_parse_array
# ./core-test_core --target test_config_parse_vector
# ./core-test_core --target test_config_parse_matrix
# ./core-test_core --target test_config_parser_full_example
# ---- Algebra
# ./core-test_core --target test_sign
# ./core-test_core --target test_linspace
# ./core-test_core --target test_linspace_timestamps
# ---- Linear Algebra
# ./core-test_core --target test_zeros
# ./core-test_core --target test_I
# ./core-test_core --target test_ones
# ./core-test_core --target test_hstack
# ./core-test_core --target test_vstack
# ./core-test_core --target test_dstack
# ./core-test_core --target test_skew
# ./core-test_core --target test_skewsq
# ./core-test_core --target test_enforce_psd
# ./core-test_core --target test_nullspace
# ./core-test_core --target test_covar_recover
# ---- Geometry
# ./core-test_core --target test_deg2rad_rad2deg
# ./core-test_core --target test_wrap180
# ./core-test_core --target test_wrap360
# ./core-test_core --target test_cross_track_error
# ./core-test_core --target test_point_left_right
# ./core-test_core --target test_closest_point
# ./core-test_core --target test_latlon_offset
# ./core-test_core --target test_latlon_diff
# ./core-test_core --target test_latlon_dist
# ---- Statistics
# ./core-test_core --target test_median
# ./core-test_core --target test_mvn
# ./core-test_core --target test_gauss_normal
# ---- Transform
# ./core-test_core --target test_tf_rot
# ./core-test_core --target test_tf_trans
# ---- Time
# ./core-test_core --target test_ts2sec
# ./core-test_core --target test_ns2sec
# ./core-test_core --target test_tic_toc
# ---- Network
# ./core-test_core --target test_tcp_server
# ./core-test_core --target test_tcp_client
# ./core-test_core --target test_tcp_server_config
# ./core-test_core --target test_tcp_client_config
# ./core-test_core --target test_tcp_server_client_loop
# ---- Interpolation
# ./core-test_core --target test_lerp
# ./core-test_core --target test_slerp
# ./core-test_core --target test_interp_pose
# ./core-test_core --target test_interp_poses
# ./core-test_core --target test_closest_poses
# ./core-test_core --target test_intersection
# ./core-test_core --target test_lerp_timestamps
# ./core-test_core --target test_lerp_data
# ./core-test_core --target test_lerp_data2
# ./core-test_core --target test_lerp_data3
# ./core-test_core --target test_ctraj
# ./core-test_core --target test_ctraj_get_pose
# ./core-test_core --target test_ctraj_get_velocity
# ./core-test_core --target test_ctraj_get_acceleration
# ./core-test_core --target test_ctraj_get_angular_velocity
# ./core-test_core --target test_sim_imu_measurement
# ---- Control
# ./core-test_core --target test_pid_construct
# ./core-test_core --target test_pid_setup
# ./core-test_core --target test_pid_update
# ./core-test_core --target test_pid_reset
# ./core-test_core --target test_carrot_ctrl_constructor
# ./core-test_core --target test_carrot_ctrl_configure
# ./core-test_core --target test_carrot_ctrl_closest_point
# ./core-test_core --target test_carrot_ctrl_carrot_point
# ./core-test_core --target test_carrot_ctrl_update
# ---- Measurements
# ./core-test_core --target test_imu_meas
# ./core-test_core --target test_imu_data
# ./core-test_core --target test_imu_data_add
# ./core-test_core --target test_imu_data_size
# ./core-test_core --target test_imu_data_last_ts
# ./core-test_core --target test_imu_data_clear
# ---- Model
# ./core-test_core --target test_two_wheel_constructor
# ./core-test_core --target test_two_wheel_update
# ./core-test_core --target test_mav_model_constructor
# ./core-test_core --target test_mav_model_update
# ---- Vision
# ./core-test_core --target test_feature_mask
# ./core-test_core --target test_grid_fast
# ./core-test_core --target benchmark_grid_fast
# ./core-test_core --target test_grid_good
# ./core-test_core --target test_radtan_distort_point
# ./core-test_core --target test_radtan_undistort_point
# ./core-test_core --target test_equi_distort_point
# ./core-test_core --target test_equi_undistort_point
# ./core-test_core --target test_pinhole
# ./core-test_core --target test_pinhole_K
# ./core-test_core --target test_pinhole_focal
# ./core-test_core --target test_pinhole_project
# ---- Simulation
# ./core-test_core --target test_sim_circle_trajectory

# -- dataset
# ./dataset-test_euroc
# ./dataset-test_kitti

# -- estimation
# ./estimation-test_dense
# ./estimation-test_factor
# ./estimation-test_factor --target test_pose_factor_jacobians
# ./estimation-test_factor --target test_extrinsic_factor_jacobians
# ./estimation-test_factor --target test_speed_bias_factor_jacobians
# ./estimation-test_factor --target test_camera_params_factor_jacobians
# ./estimation-test_factor --target test_ba_factor_jacobians
# ./estimation-test_factor --target test_cam_factor_jacobians
# ./estimation-test_factor --target test_imu_factor_jacobians
# ./estimation-test_factor --target test_imu_propagate
# ./estimation-test_factor --target test_graph
# ./estimation-test_factor --target test_graph_add_pose
# ./estimation-test_factor --target test_graph_add_landmark
# ./estimation-test_factor --target test_graph_add_camera
# ./estimation-test_factor --target test_graph_add_speed_bias
# ./estimation-test_factor --target test_graph_add_pose_factor
# ./estimation-test_factor --target test_graph_add_ba_factor
# ./estimation-test_factor --target test_graph_add_cam_factor
# ./estimation-test_factor --target test_graph_add_imu_factor
# ./estimation-test_factor --target test_graph_rm_param
# ./estimation-test_factor --target test_graph_rm_factor
# ./estimation-test_factor --target test_graph_eval
# ./estimation-test_factor --target test_graph_get_state
# ./estimation-test_factor --target test_graph_set_state
# ./estimation-test_factor --target test_graph_solve_ba
# ./estimation-test_factor --target test_swf_add_imu
# ./estimation-test_factor --target test_swf_add_camera
# ./estimation-test_factor --target test_swf_add_extrinsics
# ./estimation-test_factor --target test_swf_add_feature
# ./estimation-test_factor --target test_swf_add_speed_bias
# ./estimation-test_factor --target test_swf_add_pose_prior
# ./estimation-test_factor --target test_swf_add_ba_factor
./estimation-test_factor --target test_swf_add_imu_factor
# ./estimation-test_factor --target test_swf_add_cam_factor
# ./estimation-test_factor --target test_swf_add_marginalize
# ./estimation-test_factor --target test_swf_solve_vo
# debug ./estimation-test_factor --target test_swf_solve_vio
# ./estimation-test_frontend

# -- viz
# ./viz-test_gui
# ./viz-test_plane
