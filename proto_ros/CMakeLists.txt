CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
PROJECT(proto_ros)

# DEPENDENCIES
CMAKE_POLICY(SET CMP0054 NEW)
FIND_PACKAGE(gazebo REQUIRED QUIET)
FIND_PACKAGE(
  catkin
  REQUIRED
    roscpp
    rospy
    rosbag
    message_generation
    std_msgs
    geometry_msgs
    sensor_msgs
    message_runtime
    image_transport
    cv_bridge
)
INCLUDE_DIRECTORIES(src)
INCLUDE_DIRECTORIES(${catkin_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${GAZEBO_INCLUDE_DIRS})
INCLUDE(cmake/ImportProto.cmake)
FIND_PACKAGE(Proto REQUIRED)
INCLUDE_DIRECTORIES(${PROTO_INCLUDE_DIRS})

# LINK
SET(
  DEPS
  proto_ros
  proto
  ${catkin_LIBRARIES}
  ${PROTO_LIBRARIES}
)

# CATKING PACKAGE
CATKIN_PACKAGE(
  CATKIN_DEPENDS
    roscpp
    rospy
    rosbag
    message_generation
    std_msgs
    geometry_msgs
    sensor_msgs
    message_runtime
    image_transport
    cv_bridge
  LIBRARIES
    proto_ros
)

# PROTO ROS LIB
ADD_LIBRARY(proto_ros src/ros/ros.cpp)
INSTALL(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# GAZEBO
ADD_LIBRARY(proto_camera_plugin SHARED src/gazebo/camera_plugin.cpp)
TARGET_LINK_LIBRARIES(proto_camera_plugin ${DEPS})

ADD_LIBRARY(proto_gimbal_plugin SHARED src/gazebo/gimbal_plugin.cpp)
TARGET_LINK_LIBRARIES(proto_gimbal_plugin ${DEPS})

ADD_LIBRARY(proto_landing_target_plugin SHARED src/gazebo/landing_target_plugin.cpp)
TARGET_LINK_LIBRARIES(proto_landing_target_plugin ${DEPS})

ADD_LIBRARY(proto_mav_plugin SHARED src/gazebo/mav_plugin.cpp)
TARGET_LINK_LIBRARIES(proto_mav_plugin ${DEPS})

ADD_LIBRARY(proto_pose_plugin SHARED src/gazebo/pose_plugin.cpp)
TARGET_LINK_LIBRARIES(proto_pose_plugin ${DEPS})

# ROS NODES
# ADD_EXECUTABLE(atl_node src/ros/nodes/atl_node.cpp)
# TARGET_LINK_LIBRARIES(atl_node ${DEPS})

ADD_EXECUTABLE(example_node src/ros/nodes/example_node.cpp)
TARGET_LINK_LIBRARIES(example_node ${DEPS})

# ADD_EXECUTABLE(lz_detector_node src/ros/nodes/lz_detector_node.cpp)
# TARGET_LINK_LIBRARIES(lz_detector_node ${DEPS})
