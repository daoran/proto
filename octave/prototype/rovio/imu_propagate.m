function imu = imu_propagate(imu, a_m, w_m, dt)
  % Attitude
  w = (w_m - imu.b_g) * dt;
  imu.q_IG += 0.5 * Omega(w) * imu.q_IG;
  imu.q_IG = quat_normalize(imu.q_IG);

  % Gyro bias
  imu.b_g = imu.b_g;

  % Velocity
  C_IG = quat2rot(imu.q_IG);
  imu.v_IG += (C_IG * (a_m - imu.b_a)) * dt;

  % Accel bias
  imu.b_a = imu.b_a;

  % Position
  imu.p_IG += imu.v_IG * dt;
endfunction
