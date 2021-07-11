function [F, r_j, v_j, C_j, ba_j, bg_j] = imu_euler_integrate(ts_i, ts_j,
                                                              acc_i, gyr_i,
                                                              acc_j, gyr_j,
                                                              r_i, v_i, C_i, ba_i, bg_i, g)
  % Propagate IMU state using Euler method
  dt = ts_j - ts_i;
  dt_sq = dt * dt;

  r_j = r_i + (v_i * dt) + (0.5 * C_i * (acc_i - ba_i) * dt_sq);
  v_j = v_i + C_i * (acc_i - ba_i) * dt;
  C_j = C_i * Exp((gyr_i - bg_i) * dt);
  ba_j = ba_i;
  bg_j = bg_i;

  % Continuous time transition matrix F
  F = zeros(15, 15);
  F(1:3, 4:6) = eye(3);
  F(4:6, 7:9) = -C_j * skew(acc_i - ba_i);
  F(4:6, 10:12) = -C_j;
  F(7:9, 7:9) = -skew(gyr_i - bg_i);
  F(7:9, 13:15) = -eye(3);

  % F = eye(15) + F * dt;
endfunction
