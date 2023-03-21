#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration


def ros_gz_bridge(topic, ros_type, gz_type, direction):
    param = f'{topic}@{ros_type}{direction}{gz_type}'
    cmd = ['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge', param]
    return ExecuteProcess(cmd=cmd, output='screen')

def ros_gz_image_bridge(topic):
    ros_type = "sensor_msgs/msg/Image"
    gz_type = "gz.msgs.Image"
    direction = "["
    return ros_gz_bridge(topic, ros_type, gz_type, direction)

def ros_gz_gimbal_joint_bridge(topic):
    ros_type = "std_msgs/msg/Float64"
    gz_type = "gz.msgs.Double"
    direction = "]"
    return ros_gz_bridge(topic, ros_type, gz_type, direction)

def generate_launch_description():
    # Settings
    gz_world = "calibration.sdf"

    # Set gazebo environment variables
    pkg_share_dir = get_package_share_directory('proto_ros2')
    models_path = pkg_share_dir + "/gazebo/models"
    worlds_path = pkg_share_dir + "/gazebo/worlds"
    os.environ['GZ_SIM_RESOURCE_PATH'] = models_path + ":" + worlds_path

    # Gazebo Simulator
    gz_proc = ExecuteProcess(cmd=['gz', 'sim', gz_world, '-v'], output='screen')

    # Gazebo -> ROS2 bridges
    cam0_bridge = ros_gz_image_bridge("/gimbal/camera0")
    cam1_bridge = ros_gz_image_bridge("/gimbal/camera1")
    yaw_bridge = ros_gz_gimbal_joint_bridge("/gimbal/joint0")
    roll_bridge = ros_gz_gimbal_joint_bridge("/gimbal/joint1")
    pitch_bridge = ros_gz_gimbal_joint_bridge("/gimbal/joint2")

    # Launch
    descs = []
    descs.append(gz_proc)
    descs.append(cam0_bridge)
    descs.append(cam1_bridge)
    descs.append(yaw_bridge)
    descs.append(roll_bridge)
    descs.append(pitch_bridge)

    return LaunchDescription(descs)
