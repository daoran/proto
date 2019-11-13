#include <signal.h>

#include "proto/proto.hpp"
#include "proto/ros/node.hpp"
#include "proto/ros/bag.hpp"

using namespace proto;

// GLOBAL VARS
rosbag::Bag bag;
std::string cam0_topic;
std::string body0_topic;
std::string target0_topic;

void signal_handler(int sig) {
  bag.close();
  ros::shutdown();
}

void image_callback(const sensor_msgs::ImageConstPtr &msg) {
  // Convert ROS message to cv::Mat
  const cv::Mat image = msg_convert(msg);
  const auto ts = msg->header.stamp;
  cv::imshow("Camera View", image);

  // Capture
  char key = (char) cv::waitKey(1);
  if (key == 'c') {
    std::cout << "Capturing" << std::endl;
    bag.write(cam0_topic, ts, msg);
  }
}

void body_callback(const geometry_msgs::PoseStampedConstPtr &msg) {
  bag.write(body0_topic, msg->header.stamp, msg);
}

void target_callback(const geometry_msgs::PoseStampedConstPtr &msg) {
  bag.write(target0_topic, msg->header.stamp, msg);
}

int main(int argc, char *argv[]) {
  // Setup ROS Node
  signal(SIGINT, signal_handler);
  const std::string node_name = ros_node_name(argc, argv);
  if (ros::isInitialized() == false) {
    ros::init(argc, argv, node_name, ros::init_options::NoSigintHandler);
  }

  // Get ROS params
  ros::NodeHandle ros_nh;
  std::string rosbag_path;
  ROS_PARAM(ros_nh, node_name + "/rosbag_path", rosbag_path);
  ROS_PARAM(ros_nh, node_name + "/cam0_topic", cam0_topic);
  ROS_PARAM(ros_nh, node_name + "/body0_topic", body0_topic);
  ROS_PARAM(ros_nh, node_name + "/target0_topic", target0_topic);

  // Setup ROS bag
  bag.open(rosbag_path, rosbag::bagmode::Write);

  // Setup subscribers
  const auto cam0_sub = ros_nh.subscribe(cam0_topic, 1, image_callback);
  const auto body0_sub = ros_nh.subscribe(body0_topic, 1, body_callback);
  const auto target0_sub = ros_nh.subscribe(target0_topic, 1, target_callback);

  // Loop
  ros::spin();

  return 0;
}
