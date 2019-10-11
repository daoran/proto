#include <proto/core/core.hpp>
#include <proto/mav/atl.hpp>

#include <sensor_msgs/Joy.h>

#include "proto/ros/node.hpp"
#include "proto/ros/msg.hpp"

// NODE SETTINGS
static const double NODE_RATE = 1000.0;

// PUBLISH TOPICS
#define MOTOR_INPUTS_TOPIC "/mav/motor_inputs"

// SUBSCRIBE TOPICS
#define SET_MODE_TOPIC "/mav/mode/set"
#define SET_POSITION_TOPIC "/mav/position/set"
#define POSE_TOPIC "/mav/pose"
#define TWIST_TOPIC "/mav/twist"
#define REL_POSE_TOPIC "/landing_zone/relative_pose"
#define DETECTED_TOPIC "/landing_zone/detected"

namespace proto {

struct atl_node_t : ros_node_t {
  proto::mat4_t T_WB = proto::I(4);
  proto::vec3_t v_WB = proto::zeros(3, 1);
  proto::vec3_t w_WB = proto::zeros(3, 1);
  proto::atl_t atl;
  struct timespec last_updated = {0, 0};

  bool lz_detected = false;
  proto::mat4_t T_CZ = proto::I(4);

  atl_node_t(int argc, char **argv) : ros_node_t(argc, argv) {}

  int configure(const int hz) {
    // ROS
    // clang-format off
    ros_node_t::configure(hz);
    add_publisher<sensor_msgs::Joy>(MOTOR_INPUTS_TOPIC);
    add_subscriber(SET_MODE_TOPIC, &atl_node_t::set_mode_callback, this);
    add_subscriber(SET_POSITION_TOPIC, &atl_node_t::set_position_callback, this);
    add_subscriber(POSE_TOPIC, &atl_node_t::pose_callback, this);
    add_subscriber(TWIST_TOPIC, &atl_node_t::twist_callback, this);
    add_subscriber(REL_POSE_TOPIC, &atl_node_t::relative_pose_callback, this);
    add_subscriber(DETECTED_TOPIC, &atl_node_t::detected_callback, this);
    add_loop_callback(std::bind(&atl_node_t::loop_callback, this));
    // clang-format on

    // ATL
    std::string config;
    ROS_PARAM(ros_nh_, node_name_ + "/config", config);
    if (atl_configure(atl, config) != 0) {
      ROS_FATAL("Failed to configure ATL!");
    }

    // Start timer
    last_updated = proto::tic();

    return 0;
  }

  void publish_motor_inputs(const proto::vec4_t &u) {
    sensor_msgs::Joy msg;
    msg.axes.push_back(u(0));
    msg.axes.push_back(u(1));
    msg.axes.push_back(u(2));
    msg.axes.push_back(u(3));
    ros_pubs_[MOTOR_INPUTS_TOPIC].publish(msg);
  }

  void set_mode_callback(const std_msgs::String &msg) {
    const std::string mode = msg.data;
    if (mode == "HOVER_MODE") {
      atl.mode = HOVER_MODE;
    } else if (mode == "DISCOVER_MODE") {
      atl.mode = DISCOVER_MODE;
    } else if (mode == "TRACKING_MODE") {
      atl.mode = TRACKING_MODE;
    } else if (mode == "LANDING_MODE") {
      atl.mode = LANDING_MODE;
    } else {
      ROS_ERROR("Invalid ATL mode [%s]!", mode.c_str());
    }
  }

  void set_position_callback(const geometry_msgs::Vector3 &msg) {
    atl.position_setpoint = proto::vec3_t{msg.x, msg.y, msg.z};
  }

  void pose_callback(const geometry_msgs::PoseStamped &msg) {
    const proto::vec3_t r_WB = msg_convert(msg.pose.position);
    const proto::quat_t q_WB = msg_convert(msg.pose.orientation);
    T_WB = proto::tf(q_WB, r_WB);
  }

  void twist_callback(const geometry_msgs::TwistStamped &msg) {
    v_WB = msg_convert(msg.twist.linear);
    w_WB = msg_convert(msg.twist.angular);
  }

  void relative_pose_callback(const geometry_msgs::PoseStamped &msg) {
    const proto::vec3_t r_CZ = msg_convert(msg.pose.position);
    const proto::quat_t q_CZ = msg_convert(msg.pose.orientation);
    T_CZ = proto::tf(q_CZ, r_CZ);
  }

  void detected_callback(const std_msgs::Bool &msg) {
    lz_detected = msg.data;
  }

  int loop_callback() {
    // Calculate dt
    const double dt = proto::toc(&last_updated);
    last_updated = proto::tic();

    // Step ATL
    proto::lz_t lz{lz_detected, atl.T_BC0, T_CZ};
    proto::vec4_t u;
    atl_step(atl, T_WB, lz, dt, u);

    // Publish motor inputs
    publish_motor_inputs(u);

    return 0;
  }
};

} // namespace proto
RUN_ROS_NODE_RATE(proto::atl_node_t, NODE_RATE);
