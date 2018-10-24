#include "prototype/driver/imu/imu_base.hpp"

namespace prototype {

// void complementary_filter(const Gyroscope gyro,
//                           const Accelerometer accel,
//                           const double dt,
//                           double &roll,
//                           double &pitch) {
//
//   // Calculate pitch and roll using gyroscope
//   const double gyro_roll = (gyro.x * dt) + roll;
//   const double gyro_pitch = (gyro.y * dt) + pitch;
//
//   // Calculate pitch and roll using accelerometer
//   const double x = accel.x;
//   const double y = accel.y;
//   const double z = accel.z;
//   const double accel_pitch = (atan(x / sqrt(y * y + z * z))) * 180 / M_PI;
//   const double accel_roll = (atan(y / sqrt(x * y + z * z))) * 180 / M_PI;
//
//   // Complimentary filter
//   pitch = (0.98 * gyro_pitch) + (0.02 * accel_pitch);
//   roll = (0.98 * gyro_roll) + (0.02 * accel_roll);
// }

void IMUBase::calibrate() {
  // Let it stablize for a while first
  LOG_INFO("Calibrating IMU - DO NOT MOVE!");
  sleep(1);

  // Calculate offset
  double ax_sum, ay_sum, az_sum = 0.0;
  double gx_sum, gy_sum, gz_sum = 0.0;
  const double nb_samples = 10000;
  for (int i = 0; i < (int) nb_samples; i++) {
    this->getData();
    ax_sum += this->accel.x;
    ay_sum += this->accel.y;
    az_sum += this->accel.z;
    gx_sum += this->gyro.x;
    gy_sum += this->gyro.y;
    gz_sum += this->gyro.z;
  }

  // this->accel.offset_x = ax_sum / nb_samples;
  // this->accel.offset_y = ay_sum / nb_samples;
  // this->accel.offset_z = (az_sum / nb_samples) - 9.81;

  this->gyro.offset_x = gx_sum / nb_samples;
  this->gyro.offset_y = gy_sum / nb_samples;
  this->gyro.offset_z = gz_sum / nb_samples;

  std::cout << "accel offsets: ";
  std::cout << this->accel.offset_x << "\t";
  std::cout << this->accel.offset_y << "\t";
  std::cout << this->accel.offset_z << std::endl;
  std::cout << "gyro offsets: ";
  std::cout << this->gyro.offset_x << "\t";
  std::cout << this->gyro.offset_y << "\t";
  std::cout << this->gyro.offset_z << std::endl;

  LOG_INFO("Finished calibrating IMU!");
}

void IMUBase::recordHeader(FILE *output_file) {
  fprintf(output_file, "gyro.x,");
  fprintf(output_file, "gyro.y,");
  fprintf(output_file, "gyro.z,");

  fprintf(output_file, "accel.x,");
  fprintf(output_file, "accel.y,");
  fprintf(output_file, "accel.z\n");
}

void IMUBase::recordData(FILE *output_file) {
  fprintf(output_file, "%f,", this->gyro.x);
  fprintf(output_file, "%f,", this->gyro.y);
  fprintf(output_file, "%f,", this->gyro.z);

  fprintf(output_file, "%f,", this->accel.x);
  fprintf(output_file, "%f,", this->accel.y);
  fprintf(output_file, "%f\n", this->accel.z);
}

int IMUBase::record(std::string output_path, int nb_samples) {
  // setup
  FILE *output_file = fopen(output_path.c_str(), "w");
  this->recordHeader(output_file);

  // record
  for (int i = 0; i < nb_samples; i++) {
    // get data
    int retval = this->getData();
    if (retval == -1) {
      LOG_ERROR("failed to obtain data from IMU!");
      return -1;
    }

    // record data
    this->recordData(output_file);
    if (retval == -1) {
      LOG_ERROR("failed to record IMU data!");
      return -1;
    }
  }

  // clean up
  fclose(output_file);

  return 0;
}

} //  namespace prototype
