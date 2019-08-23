#ifndef PROTO_ROS_MSGS_HPP
#define PROTO_ROS_MSGS_HPP

#include <proto/core/core.hpp>

#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <sensor_msgs/Joy.h>
#include <sensor_msgs/NavSatFix.h>

#include <cv_bridge/cv_bridge.h>


namespace proto {

std_msgs::UInt8 msg_build(const uint8_t i);
std_msgs::Bool msg_build(const bool b);
std_msgs::String msg_build(const std::string &s);
std_msgs::Float64 msg_build(const double d);

geometry_msgs::Vector3 msg_build(const proto::vec3_t &vec);
geometry_msgs::Quaternion msg_build(const proto::quat_t &q);
geometry_msgs::PoseStamped msg_build(const size_t seq,
                                     const ros::Time &time,
                                     const std::string &frame_id,
                                     const proto::mat4_t &pose);
geometry_msgs::TwistStamped msg_build(const size_t seq,
                                      const ros::Time &time,
                                      const std::string &frame_id,
                                      const proto::vec3_t &linear_velocity,
                                      const proto::vec3_t &angular_velocity);

void msg_convert(const std_msgs::Header &msg,
                 size_t seq,
                 proto::timestamp_t &ts,
                 std::string &frame_id);
uint8_t msg_convert(const std_msgs::UInt8 &msg);
bool msg_convert(const std_msgs::Bool &msg);
float msg_convert(const std_msgs::Float64 &msg);
std::string msg_convert(const std_msgs::String &msg);

proto::vec3_t msg_convert(const geometry_msgs::Vector3 &msg);
proto::vec3_t msg_convert(const geometry_msgs::Point &msg);
proto::quat_t msg_convert(const geometry_msgs::Quaternion &msg);
cv::Mat msg_convert(const sensor_msgs::ImageConstPtr &msg, const int cv_type=CV_8UC3);

} // namespace proto
#endif // PROTO_ROS_MSGS_HPP
