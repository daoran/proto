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
  UNUSED(sig);
  bag.close();
  ros::shutdown();
}

void image_cb(const sensor_msgs::ImageConstPtr &msg) {
  // Convert ROS message to cv::Mat
  const cv::Mat image = msg_convert(msg);
  const auto ts = msg->header.stamp;
  cv::imshow("Camera View", image);

  // Capture
  char key = (char) cv::waitKey(1);
  if (key == 'c') {
    std::cout << "Capturing" << std::endl;
    bag.write("/cam0/image", ts, msg);
  }
}

void body_pose_cb(const geometry_msgs::PoseStampedConstPtr &msg) {
  bag.write("/body0/pose", msg->header.stamp, msg);
}

void body_pose_covar_cb(
    const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg) {
  geometry_msgs::PoseStamped pose;
  pose.header = msg->header;
  pose.pose = msg->pose.pose;
  bag.write("/body0/pose", msg->header.stamp, pose);
}

void body_odom_cb(const nav_msgs::OdometryConstPtr &msg) {
  geometry_msgs::PoseStamped pose;
  pose.header = msg->header;
  pose.pose = msg->pose.pose;
  bag.write("/body0/pose", msg->header.stamp, pose);
}

void target_pose_cb(const geometry_msgs::PoseStampedConstPtr &msg) {
  bag.write("/target0/pose", msg->header.stamp, msg);
}

void target_pose_covar_cb(
    const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg) {
  geometry_msgs::PoseStamped pose;
  pose.header = msg->header;
  pose.pose = msg->pose.pose;
  bag.write("/target0/pose", msg->header.stamp, pose);
}

void target_odom_cb(const nav_msgs::OdometryConstPtr &msg) {
  geometry_msgs::PoseStamped pose;
  pose.header = msg->header;
  pose.pose = msg->pose.pose;
  bag.write("/target0/pose", msg->header.stamp, pose);
}

int main(int argc, char *argv[]) {
  // Setup ROS Node
  signal(SIGINT, signal_handler);
  const std::string node_name = ros_node_name(argc, argv);
  if (ros::isInitialized() == false) {
    ros::init(argc, argv, node_name, ros::init_options::NoSigintHandler);
  }

  // Get ROS params
  ros::NodeHandle nh;
  std::string rosbag_path;
  std::string body0_topic_type;
  std::string target0_topic_type;
  ROS_PARAM(nh, node_name + "/rosbag_path", rosbag_path);
  ROS_PARAM(nh, node_name + "/cam0_topic", cam0_topic);
  ROS_PARAM(nh, node_name + "/body0_topic", body0_topic);
  ROS_PARAM(nh, node_name + "/body0_topic_type", body0_topic_type);
  ROS_PARAM(nh, node_name + "/target0_topic", target0_topic);
  ROS_PARAM(nh, node_name + "/target0_topic_type", target0_topic_type);

  // Setup ROS bag
  bag.open(rosbag_path, rosbag::bagmode::Write);

  // Setup subscribers
  // -- Camera subscriber
  const auto cam0_sub = nh.subscribe(cam0_topic, 100, image_cb);
  // -- Mocap body subscriber
  ros::Subscriber body0_sub;
  if (body0_topic_type == "geometry_msgs/PoseStamped") {
    body0_sub = nh.subscribe(body0_topic, 100, body_pose_cb);
  } else if (body0_topic_type == "geometry_msgs/PoseWithCovarianceStamped") {
    body0_sub = nh.subscribe(body0_topic, 100, body_pose_covar_cb);
  } else if (body0_topic_type == "nav_msgs/Odometry") {
    body0_sub = nh.subscribe(body0_topic, 100, body_odom_cb);
  } else {
    FATAL("Unsupported body0_topic_type [%s]!", body0_topic_type.c_str());
  }
  // -- Mocap target subscriber
  ros::Subscriber target0_sub;
  if (target0_topic_type == "geometry_msgs/PoseStamped") {
    target0_sub = nh.subscribe(target0_topic, 100, target_pose_cb);
  } else if (target0_topic_type == "geometry_msgs/PoseWithCovarianceStamped") {
    target0_sub = nh.subscribe(target0_topic, 100, target_pose_covar_cb);
  } else if (target0_topic_type == "nav_msgs/Odometry") {
    target0_sub = nh.subscribe(target0_topic, 100, target_odom_cb);
  } else {
    FATAL("Unsupported target0_topic_type [%s]!", target0_topic_type.c_str());
  }

  // Loop
  ros::spin();

  return 0;
}
