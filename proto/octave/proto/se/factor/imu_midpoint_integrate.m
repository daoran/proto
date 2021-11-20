function [F, r_j, v_j, C_j, ba_j, bg_j] = imu_midpoint_integrate(ts_i, ts_j,
                                                                 acc_i, gyr_i,
                                                                 acc_j, gyr_j,
                                                                 r_i, v_i, C_i,
                                                                 ba_i, bg_i, g)
  % Timestep
  dt = ts_j - ts_i;
  dt_sq = dt * dt;

  % Update position, velocity and orientation
  w_ij = 0.5 * (gyr_i + gyr_j) - bg_i;
  C_j = C_i * Exp(w_ij * dt);

  a_i = C_i * (acc_i - ba_i);
  a_j = C_j * (acc_j - ba_i);
  a_ij = 0.5 * (a_i + a_j);

  r_j = r_i + (v_i * dt) + (0.5 * a_ij * dt_sq);
  v_j = v_i + a_ij * dt;
  ba_j = ba_i;
  bg_j = bg_i;

  % Setup
  w_x = 0.5 * (gyr_i + gyr_j) - bg_i;
  a_i_x = acc_i - ba_i;
  a_j_x = acc_j - ba_i;
  C_w_x = skew(w_x);
  C_a_i_x = skew(a_i_x);
  C_a_j_x = skew(a_j_x);

  % Transition matrix F
  F = zeros(15, 15);
  % -- First row block
  F(1:3, 1:3) = eye(3);
  F(1:3, 4:6) = eye(3) * dt;
  F(1:3, 7:9) = -0.25 * C_i * C_a_i_x * dt_sq + -0.25 * C_j * C_a_j_x * (eye(3) - C_w_x * dt) * dt_sq;
  F(1:3, 10:12) = -0.25 * (C_i + C_j) * dt_sq;
  F(1:3, 13:15) = -0.25 * C_j * C_a_j_x * dt_sq * -dt;
  % -- Second row block
  F(4:6, 4:6) = eye(3);
  F(4:6, 7:9) = -0.5 * C_i * C_a_i_x * dt + -0.5 * C_j * C_a_i_x * (eye(3) - C_w_x * dt) * dt;
  F(4:6, 10:12) = -0.5 * (C_i + C_j) * dt;
  F(4:6, 13:15) = -0.5 * C_j * C_a_i_x * dt * -dt;
  % -- Third row block
  F(7:9, 7:9) = eye(3) - C_w_x * dt;
  F(7:9, 13:15) = -1.0 * eye(3) * dt;
  % -- Fourth row block
  F(10:12, 10:12) = eye(3);
  % -- Fifth row block
  F(13:15, 13:15) = eye(3);

  % Noise matrix G
  G = zeros(15, 18);
  % -- First row block
  G(1:3, 1:3) =  0.25 * C_i * dt_sq;
  G(1:3, 4:6) =  0.25 * -C_j * C_a_j_x  * dt_sq * 0.5 * dt;
  G(1:3, 7:9) =  0.25 * C_j * dt_sq;
  G(1:3, 10:12) =  G(1:3, 4:6);
  % -- Second row block
  G(4:6, 4:6) =  0.5 * eye(3) * dt;
  G(4:6, 10:12) =  0.5 * eye(3) * dt;
  % -- Third row block
  G(7:9, 1:3) =  0.5 * C_i * dt;
  G(7:9, 4:6) =  0.5 * -C_j * C_a_j_x  * dt * 0.5 * dt;
  G(7:9, 7:9) =  0.5 * C_j * dt;
  G(7:9, 10:12) =  G(7:9, 4:6);
  % -- Fourth row block
  G(10:12, 13:15) = eye(3) * dt;
  % -- Fifth row block
  G(13:15, 16:18) = eye(3) * dt;
endfunction
