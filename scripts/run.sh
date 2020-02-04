set -e
export CC=/usr/bin/clang
export CXX=/usr/bin/clang++
SCRIPT_PATH="`dirname \"$0\"`"
SCRIPT_PATH="`( cd \"$SCRIPT_PATH\" && pwd )`"

debug() {
  gdb \
    -ex=run \
    -ex=bt \
    -ex="set confirm off" \
    -ex=quit \
    --args "$1" "$2"
}

# CLEAN CATKIN
# cd ~/catkin_ws/
# catkin clean
# cd -

# LIBRARY
# make deps
# time make debug
time make release
sudo make install
time make ros
# sudo make debug_install
# exit

source ~/.bashrc
cd ~/catkin_ws/ && source devel/setup.bash
roslaunch proto_ros sim_calib.launch
# roslaunch proto_ros calib_camera.launch
# roslaunch proto_ros calib_stereo.launch

# roslaunch proto_ros calib_vicon_capture.launch
# mv /data/vicon_capture.bag /data/vicon-train.bag
# roslaunch proto_ros calib_vicon_capture.launch
# mv /data/vicon_capture.bag /data/vicon-test.bag
# rm -rf /data/intel_d435i/calib_vicon_data && roslaunch proto_ros calib_vicon.launch

# roslaunch proto_ros calib_vicon.launch
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
# ./ublox_rover
# ./ublox_base_station
# ./validate_intrinsics config/validate_intrinsics.yaml
# ./validate_stereo config/validate_stereo.yaml
# exit

# cd build
# ./play

# TESTS
cd proto/build/tests
# -- calib
# valgrind --leak-check=full
# ./calib-test_aprilgrid --target test_aprilgrid_calc_relative_pose
# ./calib-test_aprilgrid --target test_aprilgrid_calc_relative_pose2
# ./calib-test_aprilgrid --target test_aprilgrid_detect
# ./calib-test_aprilgrid --target test_aprilgrid_detect2
# ./calib-test_calib_camera
# ./calib-test_calib_camera_nbv
# ./calib-test_calib_data
# ./calib-test_calib_gimbal
# ./calib-test_calib_stereo
# ./calib-test_sandbox
# ./calib-test_sandbox2
# ./calib-test_sandbox3

# -- comm
# ./comm-test_tcp

# -- control
# ./control-test_carrot_ctrl
# ./control-test_pid

# -- core
# ./core-test_core

# -- driver
# ./driver-test_camera-camera
# ./driver-test_ublox

# -- dataset
# ./dataset-test_euroc
# ./dataset-test_kitti

# -- estimation
# ./estimation-test_factor
# ./estimation-test_factor --target test_graph_update
# ./estimation-test_measurement

# -- mav
# ./mav-test_atl
# ./mav-test_att_ctrl
# ./mav-test_lz
# ./mav-test_mission
# ./mav-test_pos_ctrl

# -- model
# ./model-test_mav
# ./model-test_two_wheel

# -- vision
# ./vision-test_camera-camera_geometry
# ./vision-test_camera-equi
# ./vision-test_camera-pinhole
# ./vision-test_camera-radtan
# ./vision-test_feature2d-grid_fast
# ./vision-test_feature2d-grid_good
# ./vision-test_frontend
# ./vision-test_vision_common

# -- viz
./viz-test_gui
# ./viz-test_plane
