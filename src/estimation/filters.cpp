
void complementary_filter(const vec3_t &gyro,
                          const vec3_t &accel,
                          const double dt,
                          double &roll,
                          double &pitch) {

  // Calculate pitch and roll using gyroscope
  const double gyro_roll = (gyro(0) * dt) + roll;
  const double gyro_pitch = (gyro(1) * dt) + pitch;

  // Calculate pitch and roll using accelerometer
  const double x = accel(0);
  const double y = accel(1);
  const double z = accel(2);
  const double accel_pitch = (atan(x / sqrt(y * y + z * z))) * 180 / M_PI;
  const double accel_roll = (atan(y / sqrt(x * y + z * z))) * 180 / M_PI;

  // Complimentary filter
  pitch = (0.98 * gyro_pitch) + (0.02 * accel_pitch);
  roll = (0.98 * gyro_roll) + (0.02 * accel_roll);
}
