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

# CLEAN CATKIN
# cd ~/catkin_ws/
# catkin clean
# cd -

# LIBRARY
# make deps
# time make debug
time make release
# sudo make install
# time make ros
# sudo make debug_install
# exit

# source ~/.bashrc
# cd ~/catkin_ws/ && source devel/setup.bash
# roslaunch proto_ros sim_calib.launch
# roslaunch proto_ros calib_camera.launch
# roslaunch proto_ros calib_stereo.launch
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

# APPS
# cd build/apps
# ./scripts/octave/calib_data_summary
# ./calib_camera config/calib_camera.yaml
# ./calib_stereo config/calib_stereo.yaml
# ./calib_vicon_marker config/calib_vicon_marker.yaml
# ./detect_aprilgrid config/detect_aprilgrid.yaml
# ./validate_intrinsics config/validate_intrinsics.yaml
# ./validate_stereo config/validate_stereo.yaml
# exit

# cd build
# ./play

# TESTS
cd proto/build/tests
# -- calib
# valgrind --leak-check=full
# ./calib-test_aprilgrid
# ./calib-test_calib_data
# ./calib-test_calib_camera
# ./calib-test_calib_gimbal
# ./calib-test_sandbox
# ./calib-test_sandbox2
# ./calib-test_sandbox3

# -- core
# ./core-test_core
# ./core-test_camera-camera_geometry
# ./core-test_camera-equi
# ./core-test_camera-pinhole
# ./core-test_camera-radtan
# ./core-test_feature2d-grid_fast
# ./core-test_feature2d-grid_good
# ./core-test_frontend
# ./core-test_vision_common


# -- driver
# ./driver-test_camera-camera
# ./driver-test_ublox

# -- dataset
# ./dataset-test_euroc
# ./dataset-test_kitti

# -- estimation
# ./estimation-test_ba
# ./estimation-test_ba --target test_parse_keypoints_line
# ./estimation-test_ba --target test_load_keypoints
# ./estimation-test_ba --target test_ba_residuals
# ./estimation-test_ba --target test_ba_jacobian
# ./estimation-test_ba --target test_ba_update
# ./estimation-test_ba --target test_ba_cost
# ./estimation-test_ba --target test_ba_solve
# ./estimation-test_imu
# ./estimation-test_factor --target test_pose_factor_jacobians
# ./estimation-test_factor --target test_ba_factor_jacobians
# ./estimation-test_factor --target test_cam_factor_jacobians
# ./estimation-test_factor --target test_imu_factor_jacobians
# ./estimation-test_factor --target test_graph
# ./estimation-test_factor --target test_graph_add_pose
# ./estimation-test_factor --target test_graph_add_landmark
# ./estimation-test_factor --target test_graph_add_cam_params
# ./estimation-test_factor --target test_graph_add_dist_params
# ./estimation-test_factor --target test_graph_add_sb_params
# ./estimation-test_factor --target test_graph_add_pose_factor
# ./estimation-test_factor --target test_graph_add_ba_factor
# ./estimation-test_factor --target test_graph_add_cam_factor
# ./estimation-test_factor --target test_graph_add_imu_factor
# ./estimation-test_factor --target test_graph_eval
./estimation-test_factor --target test_graph_solve
# ./estimation-test_measurement
# ./estimation-test_dense

# -- viz
# ./viz-test_gui
# ./viz-test_plane
