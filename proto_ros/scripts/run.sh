# set -e

# for FILE in `find . -not -path "./.git/*" -type f`
# do
#   echo "PROCESSING [$FILE]"
#   sed -i "s/PROTO/PROTO/g" $FILE
# done

# make
cd ~/catkin_ws/
catkin build proto_ros proto_gazebo
source "$HOME/catkin_ws/devel/setup.bash"
# roslaunch proto_ros bag2ds.launch
# roslaunch proto_ros camera.launch
# roslaunch proto_ros calib_validate_mono.launch
# roslaunch proto_ros drone_hunt.launch

export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:$HOME/catkin_ws/devel/lib
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:$HOME/catkin_ws/src/proto_ros/proto_gazebo/models
roslaunch proto_gazebo calib_sim.launch
# roslaunch proto_gazebo mav_sim.launch
