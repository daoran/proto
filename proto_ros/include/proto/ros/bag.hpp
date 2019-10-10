#ifndef PROTO_ROS_BAG_HPP
#define PROTO_ROS_BAG_HPP

#include <iostream>
#include <fstream>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <cv_bridge/cv_bridge.h>

#include <proto/proto.hpp>
#include "proto/ros/msgs.hpp"

namespace proto {

std::ofstream camera_init_output_file(const std::string &output_path);
std::ofstream imu_init_output_file(const std::string &output_path);
std::ofstream accel_init_output_file(const std::string &output_path);
std::ofstream gyro_init_output_file(const std::string &output_path);
void load_imu_data(const std::string &csv_file,
                   timestamps_t &timestamps,
                   vec3s_t &gyro,
                   vec3s_t &accel);
void image_message_handler(const rosbag::MessageInstance &msg,
                           const std::string &output_path,
                           std::ofstream &camera_data);
void imu_message_handler(const rosbag::MessageInstance &msg,
                         std::ofstream &imu_data);
void accel_message_handler(const rosbag::MessageInstance &msg,
                           std::ofstream &accel_csv,
                           timestamps_t &accel_ts,
                           vec3s_t &accel_data);
void gyro_message_handler(const rosbag::MessageInstance &msg,
                          std::ofstream &gyro_csv,
                          timestamps_t &gyro_ts,
                          vec3s_t &gyro_data);

}  // namespace proto
#endif  // PROTO_ROS_BAG_HPP
