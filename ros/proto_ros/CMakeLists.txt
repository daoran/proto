CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
PROJECT(proto_ros)

# DEPENDENCIES
CMAKE_POLICY(SET CMP0054 NEW)
FIND_PACKAGE(Proto REQUIRED)
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
INCLUDE_DIRECTORIES(include)
INCLUDE_DIRECTORIES(${catkin_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${GAZEBO_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${PROTO_INCLUDE_DIRS})

# LINK
SET(
  DEPS
  proto_ros
  ${PROTO_LIBRARIES}
  ${catkin_LIBRARIES}
)

# ROS MESSAGE
ADD_MESSAGE_FILES(
  DIRECTORY msgs
  FILES
  UBloxRover.msg
)
GENERATE_MESSAGES(DEPENDENCIES std_msgs)

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
  INCLUDE_DIRS
    include
  LIBRARIES
    proto_ros
)

# PROTO ROS LIB
ADD_LIBRARY(
  proto_ros
  src/ros/node.cpp
  src/ros/msgs.cpp
)
INSTALL(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# ROS NODES
ADD_EXECUTABLE(atl_node src/ros/nodes/atl_node.cpp)
TARGET_LINK_LIBRARIES(atl_node ${DEPS})

ADD_EXECUTABLE(calib_validate_mono_node src/ros/nodes/calib_validate_mono_node.cpp)
TARGET_LINK_LIBRARIES(calib_validate_mono_node ${DEPS})

ADD_EXECUTABLE(calib_validate_stereo_node src/ros/nodes/calib_validate_stereo_node.cpp)
TARGET_LINK_LIBRARIES(calib_validate_stereo_node ${DEPS})

ADD_EXECUTABLE(camera_node src/ros/nodes/camera_node.cpp)
TARGET_LINK_LIBRARIES(camera_node ${DEPS})

ADD_EXECUTABLE(example_node src/ros/nodes/example_node.cpp)
TARGET_LINK_LIBRARIES(example_node ${DEPS})

ADD_EXECUTABLE(lz_detector_node src/ros/nodes/lz_detector_node.cpp)
TARGET_LINK_LIBRARIES(lz_detector_node ${DEPS})
