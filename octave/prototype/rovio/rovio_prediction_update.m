function rovio = rovio_prediction_update(rovio, a_m, w_m, dt)
  imu = rovio.imu;

  % Accelerometer
  w_f = [0.0; 0.0; 0.0];
  f_hat = a_m - imu.b_f - w_f;

  % Gyroscope
  w_w = [0.0; 0.0; 0.0];
  w_hat = w_m - imu.b_w - w_w;

  % Position
  w_r = [0.0; 0.0; 0.0];
  dr = -skew(w_hat) * imu.r + imu.v + w_r;

  % Velocity
  dv = -skew(w_hat) * imu.v + f_hat + quat_inv(g);

  % Attitude
  dq = -quat_delta(w_hat);

  % Accel bias
  db_f = imu.b_f;

  % Gyro bias
  db_g = imu.b_g;

  % % IMU-Camera extrinsics - translation
  % w_c = [0.0; 0.0; 0.0];
  % dc = w_c;
  %
  % % IMU-Camera extrinsics - rotation
  % w_z = [1.0; 0.0; 0.0; 0.0];
  % dz = w_z;
endfunction
