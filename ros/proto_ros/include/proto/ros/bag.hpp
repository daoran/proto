#ifndef PROTO_ROS_BAG_HPP
#define PROTO_ROS_BAG_HPP

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <cv_bridge/cv_bridge.h>

#include <proto/proto.hpp>
#include "proto/ros/msgs.hpp"

namespace proto {

std::ofstream camera_init_output_file(const std::string &output_path) {
  const std::string save_path{output_path + "/data.csv"};

  // Check save dir
  if (proto::dir_exists(output_path) == false) {
     if (proto::dir_create(output_path) != 0) {
        FATAL("Failed to create dir [%s]", output_path.c_str());
     }
  }

  // Create output csv file
  std::ofstream data_file(save_path);
  if (data_file.good() == false) {
    FATAL("Failed to create output file [%s]", save_path.c_str());
  }

  // Write data file header
  data_file << "timestamp [ns],filename" << std::endl;

  return data_file;
}

std::ofstream imu_init_output_file(const std::string &output_path) {
  const std::string save_path{output_path + "/data.csv"};

  // Check save dir
  if (proto::dir_exists(output_path) == false) {
     if (proto::dir_create(output_path) != 0) {
        FATAL("Failed to create dir [%s]", output_path.c_str());
     }
  }

  // Create output csv file
  std::ofstream data_file(save_path);
  if (data_file.good() == false) {
    FATAL("Failed to create output file [%s]", save_path.c_str());
  }

  // Write data file header
  data_file << "timestamp [ns],";
  data_file << "w_RS_S_x [rad s^-1],";
  data_file << "w_RS_S_y [rad s^-1],";
  data_file << "w_RS_S_z [rad s^-1],";
  data_file << "a_RS_S_x [m s^-2],";
  data_file << "a_RS_S_y [m s^-2],";
  data_file << "a_RS_S_z [m s^-2]" << std::endl;

  return data_file;
}

std::ofstream accel_init_output_file(const std::string &output_path) {
  const std::string save_path{output_path + "/data.csv"};

  // Check save dir
  if (proto::dir_exists(output_path) == false) {
     if (proto::dir_create(output_path) != 0) {
        FATAL("Failed to create dir [%s]", output_path.c_str());
     }
  }

  // Create output csv file
  std::ofstream data_file(save_path);
  if (data_file.good() == false) {
    FATAL("Failed to create output file [%s]", save_path.c_str());
  }

  // Write data file header
  data_file << "timestamp [ns],";
  data_file << "a_RS_S_x [m s^-2],";
  data_file << "a_RS_S_y [m s^-2],";
  data_file << "a_RS_S_z [m s^-2]" << std::endl;

  return data_file;
}

std::ofstream gyro_init_output_file(const std::string &output_path) {
  const std::string save_path{output_path + "/data.csv"};

  // Check save dir
  if (proto::dir_exists(output_path) == false) {
     if (proto::dir_create(output_path) != 0) {
        FATAL("Failed to create dir [%s]", output_path.c_str());
     }
  }

  // Create output csv file
  std::ofstream data_file(save_path);
  if (data_file.good() == false) {
    FATAL("Failed to create output file [%s]", save_path.c_str());
  }

  // Write data file header
  data_file << "timestamp [ns],";
  data_file << "w_RS_S_x [rad s^-1],";
  data_file << "w_RS_S_y [rad s^-1],";
  data_file << "w_RS_S_z [rad s^-1]" << std::endl;

  return data_file;
}

void load_imu_data(const std::string &csv_file,
                   proto::timestamps_t &timestamps,
                   proto::vec3s_t &gyro,
                   proto::vec3s_t &accel) {
  // Open file for loading
  int nb_rows = 0;
  FILE *fp = proto::file_open(csv_file, "r", &nb_rows);
  if (fp == nullptr) {
    LOG_ERROR("Failed to open [%s]!", csv_file.c_str());
    return;
  }

  // Parse file
  for (int i = 0; i < nb_rows; i++) {
    // Skip first line
    if (i == 0) {
      proto::skip_line(fp);
      continue;
    }

    // Parse line
    proto::timestamp_t ts = 0;
    double w_x, w_y, w_z = 0.0;
    double a_x, a_y, a_z = 0.0;
    const int retval = fscanf(fp, "%" SCNu64 ",%lf,%lf,%lf,%lf,%lf,%lf",
                              &ts, &w_x, &w_y, &w_z, &a_x, &a_y, &a_z);
    if (retval != 6) {
      LOG_ERROR("Failed to parse line [%d] in [%s]!", i, csv_file.c_str());
      return;
    }
    timestamps.push_back(ts);
    gyro.emplace_back(w_x, w_y, w_z);
    accel.emplace_back(a_x, a_y, a_z);
  }
  fclose(fp);
}

void image_message_handler(const rosbag::MessageInstance &msg,
                           const std::string &output_path,
                           std::ofstream &camera_data) {
  const auto image_msg = msg.instantiate<sensor_msgs::Image>();
  const auto ts = ros::Time(image_msg->header.stamp);
  const auto ts_str = std::to_string(ts.toNSec());
  const std::string save_path{output_path + ts_str + ".png"};

  // Check message already processed
  if (proto::file_exists(save_path)) {
    return;
  }

  // Check save dir
  if (proto::dir_exists(output_path) == false) {
     if (proto::dir_create(output_path) != 0) {
        FATAL("Failed to create dir [%s]", output_path.c_str());
     }
  }

  // Convert ROS message to cv image
  cv_bridge::CvImagePtr bridge;
  bridge = cv_bridge::toCvCopy(image_msg, "bgr8");

  // Save image to file
  if (cv::imwrite(save_path, bridge->image) == false) {
    FATAL("Failed to save image to [%s]", save_path.c_str());
  }

  // Save image file to data.csv
  camera_data << ts.toNSec() << "," << ts.toNSec() << ".png" << std::endl;
}

void imu_message_handler(const rosbag::MessageInstance &msg,
                         std::ofstream &imu_data) {
  const auto imu_msg = msg.instantiate<sensor_msgs::Imu>();
  const auto ts = ros::Time(imu_msg->header.stamp);
  const auto ts_str = std::to_string(ts.toNSec());

  // Convert ros msg data
  const Eigen::Vector3d gyro = msg_convert(imu_msg->angular_velocity);
  const Eigen::Vector3d accel = msg_convert(imu_msg->linear_acceleration);

  // Save imu measurement to file
  // -- Timestamp [ns]
  imu_data << ts.toNSec() << ",";
  // -- Angular velocity [rad s^-1]
  imu_data << gyro(0) << ",";
  imu_data << gyro(1) << ",";
  imu_data << gyro(2) << ",";
  // -- Accelerometer [m s^-2]
  imu_data << accel(0) << ",";
  imu_data << accel(1) << ",";
  imu_data << accel(2) << std::endl;
}

void accel_message_handler(const rosbag::MessageInstance &msg,
                           std::ofstream &accel_csv,
                           proto::timestamps_t &accel_ts,
                           proto::vec3s_t &accel_data) {
  const auto accel_msg = msg.instantiate<sensor_msgs::Imu>();
  const auto ts = ros::Time(accel_msg->header.stamp);
  const auto ts_str = std::to_string(ts.toNSec());

  // Convert ros msg data
  const Eigen::Vector3d gyro = msg_convert(accel_msg->angular_velocity);
  const Eigen::Vector3d accel = msg_convert(accel_msg->linear_acceleration);

  // Save imu measurement to file
  // -- Timestamp [ns]
  accel_csv << ts.toNSec() << ",";
  // -- Accelerometer [m s^-2]
  accel_csv << accel(0) << ",";
  accel_csv << accel(1) << ",";
  accel_csv << accel(2) << std::endl;

  // Keep track of accel data
  accel_ts.push_back(ts.toNSec());
  accel_data.push_back(accel);
}

void gyro_message_handler(const rosbag::MessageInstance &msg,
                          std::ofstream &gyro_csv,
                          proto::timestamps_t &gyro_ts,
                          proto::vec3s_t &gyro_data) {
  const auto gyro_msg = msg.instantiate<sensor_msgs::Imu>();
  const auto ts = ros::Time(gyro_msg->header.stamp);
  const auto ts_str = std::to_string(ts.toNSec());

  // Convert ros msg data
  const Eigen::Vector3d gyro = msg_convert(gyro_msg->angular_velocity);
  const Eigen::Vector3d accel = msg_convert(gyro_msg->linear_acceleration);

  // Save imu measurement to file
  // -- Timestamp [ns]
  gyro_csv << ts.toNSec() << ",";
  // -- Angular velocity [rad s^-1]
  gyro_csv << gyro(0) << ",";
  gyro_csv << gyro(1) << ",";
  gyro_csv << gyro(2) << std::endl;

  // Keep track of gyro data
  gyro_ts.push_back(ts.toNSec());
  gyro_data.push_back(gyro);
}

}  // namespace proto
#endif  // PROTO_ROS_BAG_HPP
