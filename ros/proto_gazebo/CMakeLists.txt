CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
PROJECT(proto_gazebo)

# DEPENDENCIES
CMAKE_POLICY(SET CMP0054 NEW)
FIND_PACKAGE(Proto REQUIRED)
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
    proto_ros
)
INCLUDE_DIRECTORIES(include)
INCLUDE_DIRECTORIES(${catkin_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${GAZEBO_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${PROTO_INCLUDE_DIRS})

# GAZEBO
LINK_DIRECTORIES(${GAZEBO_LIBRARY_DIRS})
LIST(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

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
    proto_ros
)

# INSTALL AND DEPS
INSTALL(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
SET(
  DEPS
  ${PROTO_LIBRARIES}
  ${catkin_LIBRARIES}
)

# GAZEBO
ADD_LIBRARY(proto_camera_plugin SHARED src/ros/gazebo/camera_plugin.cpp)
TARGET_LINK_LIBRARIES(proto_camera_plugin ${DEPS})

ADD_LIBRARY(proto_gimbal_plugin SHARED src/ros/gazebo/gimbal_plugin.cpp)
TARGET_LINK_LIBRARIES(proto_gimbal_plugin ${DEPS})

ADD_LIBRARY(proto_landing_target_plugin SHARED src/ros/gazebo/landing_target_plugin.cpp)
TARGET_LINK_LIBRARIES(proto_landing_target_plugin ${DEPS})

ADD_LIBRARY(proto_mav_plugin SHARED src/ros/gazebo/mav_plugin.cpp)
TARGET_LINK_LIBRARIES(proto_mav_plugin ${DEPS})

ADD_LIBRARY(proto_pose_plugin SHARED src/ros/gazebo/pose_plugin.cpp)
TARGET_LINK_LIBRARIES(proto_pose_plugin ${DEPS})
