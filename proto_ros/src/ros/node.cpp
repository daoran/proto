#include "proto/ros/node.hpp"

namespace proto {

ros_node_t::ros_node_t() {}

ros_node_t::~ros_node_t() {
  ros::shutdown();
  if (ros_rate_) {
    delete ros_rate_;
  }
}

int ros_node_t::configure() {
  ros_nh_.getParam("/debug_mode", debug_mode_);
  ros_nh_.getParam("/sim_mode", sim_mode_);
  configured_ = true;

  return 0;
}

int ros_node_t::configure(const int hz) {
  configure();
  ros_rate_ = new ros::Rate(hz);
  return 0;
}

void ros_node_t::shutdown_callback(const std_msgs::Bool &msg) {
  if (msg.data) {
    ros::shutdown();
  }
}

int ros_node_t::add_shutdown_subscriber(const std::string &topic) {
  ros::Subscriber sub;

  // Pre-check
  if (configured_ == false) {
    return -1;
  }

  // Register subscriber
  sub = ros_nh_.subscribe(topic, 1, &ros_node_t::shutdown_callback, this);
  ros_subs_[topic] = sub;

  return 0;
}

int ros_node_t::add_image_publisher(const std::string &topic) {
  // Pre-check
  if (configured_ == false) {
    return -1;
  }

  // Image transport
  image_transport::ImageTransport it(ros_nh_);
  img_pubs_[topic] = it.advertise(topic, 1);

  return 0;
}

int ros_node_t::add_loop_callback(std::function<int()> cb) {
  loop_cb_ = cb;
  return 0;
}

int ros_node_t::loop() {
  // Pre-check
  if (configured_ == false) {
    return -1;
  }

  // Loop
  ROS_INFO("ROS node [%s] is running!", node_name_.c_str());
  while (ros::ok()) {
    // Loop callback
    if (loop_cb_ != nullptr) {
      int retval = loop_cb_();
      if (retval != 0) {
        return retval;
      }
    }

    // Update
    ros::spinOnce();
    ros_seq_++;
    ros_last_updated_ = ros::Time::now();
    if (ros_rate_) {
      ros_rate_->sleep();
    }
  }

  return 0;
}

} // namespace proto
