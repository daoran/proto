#include <string>

#include <sensor_msgs/Joy.h>

#include <proto/proto.hpp>
#include "proto/ros/node.hpp"
#include "proto/ros/msg.hpp"

// NODE SETTINGS
static const double NODE_RATE = 60.0;

// SUBSCRIBE TOPICS
#define CAMERA_TOPIC "/mav/camera"

// PUBLISH TOPICS
#define REL_POSE_TOPIC "/landing_zone/relative_pose"
#define DETECTED_TOPIC "/landing_zone/detected"

namespace proto {

struct lz_detector_node_t : ros_node_t {
  lz_detector_t lz_detector;
  pinhole_t pinhole;

  lz_detector_node_t() : ros_node_t() {}

  int configure(const int hz) {
    // ROS
    ros_node_t::configure(hz);
    add_subscriber(CAMERA_TOPIC, &lz_detector_node_t::camera_callback, this);
    add_publisher<geometry_msgs::PoseStamped>(REL_POSE_TOPIC);
    add_publisher<std_msgs::Bool>(DETECTED_TOPIC);
    add_loop_callback(std::bind(&lz_detector_node_t::loop_callback, this));

    // Configure landing zone detector
    std::string config_file;
    ROS_PARAM(ros_nh_, node_name_ + "/config", config_file);
    if (lz_detector_configure(lz_detector, config_file, "landing_zone") != 0) {
      ROS_FATAL("Failed to configure landing zone detector!");
    }

    // Configure pinhole camera model
    config_t config{config_file};
    std::string camera_model;
    vec4_t cam0_intrinsics;
    parse(config, "cam0.camera_model", camera_model);
    parse(config, "cam0.intrinsics", cam0_intrinsics);
    if (camera_model != "pinhole") {
      throw std::runtime_error("Invalid camera model !");
    }
    pinhole = pinhole_t{cam0_intrinsics};

    return 0;
  }

  void camera_callback(const sensor_msgs::ImageConstPtr &msg) {
    // Parse image message
    cv_bridge::CvImagePtr image_ptr;
    image_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    const auto image = image_ptr->image.clone();
    const auto image_ts = msg->header.stamp;

    // Detect and publish landing zone relative pose
    mat4_t T_CZ;
    int detected = lz_detector_detect(lz_detector, image, pinhole, T_CZ);
    if (detected) {
      publish_relative_pose(image_ts, T_CZ);
      publish_detected(true);
    } else {
      publish_detected(false);
    }
  }

  void publish_relative_pose(const ros::Time &ts, const mat4_t &T_CZ) {
    const auto msg = msg_build(ros_seq_, ts, "landing_target", T_CZ);
    ros_pubs_[REL_POSE_TOPIC].publish(msg);
  }

  void publish_detected(bool detected) {
    const auto msg = msg_build(detected);
    ros_pubs_[DETECTED_TOPIC].publish(msg);
  }

  int loop_callback() {
    return 0;
  }
};

} // namespace proto
RUN_ROS_NODE_RATE(proto::lz_detector_node_t, NODE_RATE);
