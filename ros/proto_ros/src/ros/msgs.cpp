#include "proto/ros/msgs.hpp"

namespace proto {

std_msgs::Bool msg_build(const bool b) {
  std_msgs::Bool msg;
  msg.data = b;
  return msg;
}

std_msgs::String msg_build(const std::string &s) {
  std_msgs::String msg;
  msg.data = s;
  return msg;
}

std_msgs::Float64 msg_build(double d) {
  std_msgs::Float64 msg;
  msg.data = d;
  return msg;
}

geometry_msgs::Vector3 msg_build(proto::vec3_t &vec) {
  geometry_msgs::Vector3 msg;
  msg.x = vec(0);
  msg.y = vec(1);
  msg.z = vec(2);
  return msg;
}

geometry_msgs::Vector3 msg_build(const proto::vec3_t &vec) {
  geometry_msgs::Vector3 msg;
  msg.x = vec(0);
  msg.y = vec(1);
  msg.z = vec(2);
  return msg;
}

geometry_msgs::Quaternion msg_build(const proto::quat_t &q) {
  geometry_msgs::Quaternion msg;
  msg.w = q.w();
  msg.x = q.x();
  msg.y = q.y();
  msg.z = q.z();
  return msg;
}

geometry_msgs::PoseStamped msg_build(const size_t seq,
                                     const ros::Time &time,
                                     const std::string &frame_id,
                                     const proto::mat4_t &pose) {
  geometry_msgs::PoseStamped msg;

  msg.header.seq = seq;
  msg.header.stamp = time;
  msg.header.frame_id = frame_id;

  proto::vec3_t position = proto::tf_trans(pose);
  msg.pose.position.x = position(0);
  msg.pose.position.y = position(1);
  msg.pose.position.z = position(2);

  proto::quat_t orientation = proto::tf_quat(pose);
  msg.pose.orientation.w = orientation.w();
  msg.pose.orientation.x = orientation.x();
  msg.pose.orientation.y = orientation.y();
  msg.pose.orientation.z = orientation.z();

  return msg;
}

geometry_msgs::TwistStamped msg_build(const size_t seq,
                                      const ros::Time &time,
                                      const std::string &frame_id,
                                      const proto::vec3_t &linear_velocity,
                                      const proto::vec3_t &angular_velocity) {
  geometry_msgs::TwistStamped msg;

  msg.header.seq = seq;
  msg.header.stamp = time;
  msg.header.frame_id = frame_id;

  msg.twist.linear.x = linear_velocity(0);
  msg.twist.linear.y = linear_velocity(1);
  msg.twist.linear.z = linear_velocity(2);

  msg.twist.angular.x = angular_velocity(0);
  msg.twist.angular.y = angular_velocity(1);
  msg.twist.angular.z = angular_velocity(2);

  return msg;
}

void msg_convert(const std_msgs::Header &msg,
                 size_t seq,
                 proto::timestamp_t &ts,
                 std::string &frame_id) {
  seq = msg.seq;
  ts = msg.stamp.toNSec();
  frame_id = msg.frame_id;
}

uint8_t msg_convert(const std_msgs::UInt8 &msg) {
  return msg.data;
}

bool msg_convert(const std_msgs::Bool &msg) {
  return msg.data;
}

float msg_convert(const std_msgs::Float64 &msg) {
  return msg.data;
}

std::string msg_convert(const std_msgs::String &msg) {
  return msg.data;
}

proto::vec3_t msg_convert(const geometry_msgs::Vector3 &msg) {
  return proto::vec3_t{msg.x, msg.y, msg.z};
}

proto::vec3_t msg_convert(const geometry_msgs::Point &msg) {
  return proto::vec3_t{msg.x, msg.y, msg.z};
}

proto::quat_t msg_convert(const geometry_msgs::Quaternion &msg) {
  return proto::quat_t{msg.w, msg.x, msg.y, msg.z};
}

cv::Mat msg_convert(const sensor_msgs::ImageConstPtr &msg, const int cv_type) {
	cv_bridge::CvImagePtr image_ptr = cv_bridge::toCvCopy(msg);
	size_t size = image_ptr->image.total() * image_ptr->image.elemSize();
	size_t rows = image_ptr->image.rows;
	size_t cols = image_ptr->image.cols;
	size_t row_bytes = size / rows;
	return cv::Mat(rows, cols, cv_type, image_ptr->image.data, row_bytes);
}

} // namespace proto
