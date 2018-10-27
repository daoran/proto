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
