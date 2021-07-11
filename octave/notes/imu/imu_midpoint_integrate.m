function [F, r_j, v_j, C_j, ba_j, bg_j] = imu_midpoint_integrate(ts_i, ts_j,
                                                                 acc_i, gyr_i,
                                                                 acc_j, gyr_j,
                                                                 r_i, v_i, C_i, ba_i, bg_i, g)
  dt = ts_j - ts_i;
  dt_sq = dt * dt;

  un_acc_i = C_i * (acc_i - ba_i);
  un_gyr = 0.5 * (gyr_i + gyr_j) - bg_i;
  C_j = C_i * Exp(un_gyr * dt);

  un_acc_j = C_j * (acc_j - ba_i);
  un_acc = 0.5 * (un_acc_i + un_acc_j);

  r_j = r_i + v_i * dt + 0.5 * un_acc * dt_sq;
  v_j = v_i + un_acc * dt;

  ba_j = ba_i;
  bg_j = bg_i;

  % Transition matrix F
  w_x = 0.5 * (gyr_i + gyr_j) - bg_i;
  a_i_x = acc_i - ba_i;
  a_j_x = acc_j - ba_i;

  R_w_x = skew(w_x);
  R_a_i_x = skew(a_i_x);
  R_a_j_x = skew(a_j_x);

  F = zeros(15, 15);
  F(1:3, 1:3) = eye(3);
  F(1:3, 4:6) = eye(3) * dt;
  F(1:3, 7:9) = -0.25 * C_i * R_a_i_x * dt_sq + -0.25 * C_j * R_a_j_x * (eye(3) - R_w_x * dt) * dt_sq;
  F(1:3, 10:12) = -0.25 * (C_i + C_j) * dt_sq;
  F(1:3, 13:15) = -0.25 * C_j * R_a_j_x * dt_sq * -dt;

  F(4:6, 4:6) = eye(3);
  F(4:6, 7:9) = -0.5 * C_i * R_a_i_x * dt + -0.5 * C_j * R_a_i_x * (eye(3) - R_w_x * dt) * dt;
  F(4:6, 10:12) = -0.5 * (C_i + C_j) * dt;
  F(4:6, 13:15) = -0.5 * C_j * R_a_i_x * dt * -dt;

  F(7:9, 7:9) = eye(3) - R_w_x * dt;
  F(7:9, 13:15) = -1.0 * eye(3) * dt;

  F(10:12, 10:12) = eye(3);
  F(13:15, 13:15) = eye(3);
endfunction
