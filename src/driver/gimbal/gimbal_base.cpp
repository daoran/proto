#include "prototype/driver/gimbal/gimbal_base.hpp"

namespace prototype {

GimbalBase::GimbalBase() {}

GimbalBase::~GimbalBase() { this->off(); }

int GimbalBase::configure(const std::string &config_file) {
  // Parse config file
  config_parser_t parser;
  std::string device_path;
  config_parser_add(parser, "device_path", &device_path);
  config_parser_add(parser, "gimbal_limits.roll_min", &this->roll_limits[0]);
  config_parser_add(parser, "gimbal_limits.roll_max", &this->roll_limits[1]);
  config_parser_add(parser, "gimbal_limits.pitch_min", &this->pitch_limits[0]);
  config_parser_add(parser, "gimbal_limits.pitch_max", &this->pitch_limits[1]);
  if (config_parser_load(parser, config_file) != 0) {
    LOG_ERROR("Failed to load config file [%s]!", config_file.c_str());
    return -1;
  }

  // Convert degrees to radians
  this->roll_limits[0] = deg2rad(this->roll_limits[0]);
  this->roll_limits[1] = deg2rad(this->roll_limits[1]);
  this->pitch_limits[0] = deg2rad(this->pitch_limits[0]);
  this->pitch_limits[1] = deg2rad(this->pitch_limits[1]);

  // SBGC
  this->sbgc = SBGC(device_path);
  if (this->sbgc.connect() != 0 || this->sbgc.on() != 0) {
    LOG_ERROR("Failed to connect or switch on SBGC!");
    return -2;
  }

  this->configured = true;
  return 0;
}

int GimbalBase::on() { return this->sbgc.on(); }

int GimbalBase::off() { return this->sbgc.off(); }

int GimbalBase::update() {
  int retval = this->sbgc.getRealtimeData4();
  if (retval != 0) {
    LOG_ERROR("Failed to get data from SBGC!");
    return -1;
  }

  // retval = this->sbgc.getAnglesExt();
  // if (retval != 0) {
  //   return -1;
  // }

  // Convert from G's to m/s^2
  const double k_gravity = 9.80665;
  this->imu_accel(0) = this->sbgc.data.accel(0) * k_gravity;
  this->imu_accel(1) = this->sbgc.data.accel(1) * k_gravity;
  this->imu_accel(2) = this->sbgc.data.accel(2) * k_gravity;

  this->imu_gyro(0) = deg2rad(this->sbgc.data.gyro(0));
  this->imu_gyro(1) = deg2rad(this->sbgc.data.gyro(1));
  this->imu_gyro(2) = deg2rad(this->sbgc.data.gyro(2));

  this->camera_angles(0) = deg2rad(this->sbgc.data.camera_angles(0));
  this->camera_angles(1) = deg2rad(this->sbgc.data.camera_angles(1));
  this->camera_angles(2) = deg2rad(this->sbgc.data.camera_angles(2));

  this->frame_angles(0) = deg2rad(this->sbgc.data.frame_angles(0));
  this->frame_angles(1) = deg2rad(this->sbgc.data.frame_angles(1));
  this->frame_angles(2) = deg2rad(this->sbgc.data.frame_angles(2));

  this->encoder_angles(0) = deg2rad(this->sbgc.data.encoder_angles(0));
  this->encoder_angles(1) = deg2rad(this->sbgc.data.encoder_angles(1));
  this->encoder_angles(2) = deg2rad(this->sbgc.data.encoder_angles(2));

  return 0;
}

int GimbalBase::setAngle(const double roll, const double pitch) {
  this->setpoints(0) = roll * 180 / M_PI;
  this->setpoints(1) = pitch * 180 / M_PI;
  this->setpoints(2) = 0.0 * 180 / M_PI;

  return this->sbgc.setAngle(this->setpoints(0),
                             this->setpoints(1),
                             this->setpoints(2));
}

void GimbalBase::printSetpoints() {
  std::cout << "roll setpoint: " << this->setpoints(0) << "\t";
  std::cout << "pitch setpoint: " << this->setpoints(1) << std::endl;
}

} //  namespace prototype
