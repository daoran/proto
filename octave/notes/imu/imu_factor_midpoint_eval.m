function [r, jacs] = imu_factor_midpoint_eval(imu_ts, imu_acc, imu_gyr, g,
                                              pose_i, sb_i, pose_j, sb_j)
  dC = eye(3, 3);
  dq = rot2quat(dC);
  dv = zeros(3, 1);
  dr = zeros(3, 1);
  ba_i = sb_i(4:6);
  bg_i = sb_i(7:9);

  dt = 0.0;
  Dt = 0.0;

  state_F = eye(15, 15);   % State jacobian
  state_P = zeros(15, 15); % State covariance

  % Noise covariance matrix
  Q = zeros(12, 12);
  noise_acc = 0.08;    % accelerometer measurement noise standard deviation.
  noise_gyr = 0.004;   % gyroscope measurement noise standard deviation.
  noise_ba = 0.00004;  % accelerometer bias random work noise standard deviation.
  noise_bg = 2.0e-6;   % gyroscope bias random work noise standard deviation.
  Q(1:3, 1:3) = (noise_acc * noise_acc) * eye(3);
  Q(4:6, 4:6) = (noise_gyr * noise_gyr) * eye(3);
  Q(7:9, 7:9) = (noise_ba * noise_ba) * eye(3);
  Q(10:12, 10:12) = (noise_bg * noise_bg) * eye(3);

  dt = imu_ts(2) - imu_ts(1);
  dt_sq = dt * dt;
  for k = 1:(length(imu_ts)-1)
    % Calculate dt
    t = imu_ts(k);

    % Mid point integration
    ts_i = imu_ts(k);
    ts_j = imu_ts(k+1);
    acc_i = imu_acc(:, k);
    gyr_i = imu_gyr(:, k);
    acc_j = imu_acc(:, k+1);
    gyr_j = imu_gyr(:, k+1);

    [F, dr, dv, dC, ba_i, bg_i] = imu_midpoint_integrate(ts_i, ts_j,
                                                         acc_i, gyr_i,
                                                         acc_j, gyr_j,
                                                         dr, dv, dC, ba_i, bg_i, g);
    state_F = F * state_F;

    % % Continuous time input matrix G
    % G = zeros(15, 12);
    % G(4:6, 1:3) = -dC;
    % G(7:9, 4:6) = -eye(3);
    % G(10:12, 7:9) = eye(3);
    % G(13:15, 10:12) = eye(3);

    % Discretize transition matrix F by using Taylor Series to the 3rd order
    % "Quaternion kinematics for the error-state Kalman filter" (2017)
    % By Joan Sola
    % [Section B, p.73 "Closed-form Integration Methods"]
    % F_dt = F * dt;
    % Phi = eye(15);
    % Phi += F_dt;
    % Phi += (1.0 / 2.0) * F_dt^2;
    % Phi += (1.0 / 6.0) * F_dt^3;
    % state_F = Phi * state_F;

    % Propagate the state covariance matrix
    % state_P = (Phi * state_P * Phi') + (Phi * G * Q * G' * Phi' * dt);

    % % Propagate the state jacobian and covariance
    % I_F_dt = (eye(15) + F * dt);
    % G_dt = G * dt;
    % state_F = I_F_dt * state_F;
    % state_P = I_F_dt * state_P * I_F_dt' + G_dt * Q * G_dt';

    % Update time
    Dt += dt;
  endfor

  % Timestep i
  r_i = tf_trans(pose_i);
  C_i = tf_rot(pose_i);
  q_i = tf_quat(pose_i);
  v_i = sb_i(1:3);
  ba_i = sb_i(4:6);
  bg_i = sb_i(7:9);

  % Timestep j
  r_j = tf_trans(pose_j);
  C_j = tf_rot(pose_j);
  q_j = tf_quat(pose_j);
  v_j = sb_j(1:3);
  ba_j = sb_j(4:6);
  bg_j = sb_j(7:9);

  % Correct the relative position, velocity and orientation
  % -- Extract jacobians from error-state jacobian
  dr_dba = state_F(1:3, 10:12);
  dr_dbg = state_F(1:3, 13:15);
  dv_dba = state_F(4:6, 10:12);
  dv_dbg = state_F(4:6, 13:15);
  dq_dbg = state_F(7:9, 13:15);
  dba = ba_i - ba_i;
  dbg = bg_i - ba_i;
  % -- Correct the relative position, velocity and rotation
  dr = dr + dr_dba * dba + dr_dbg * dbg;
  dv = dv + dv_dba * dba + dv_dbg * dbg;
  dq = quat_mul(quat_delta(dq_dbg * dbg), rot2quat(dC));
  dC = dC * Exp(dq_dbg * dbg);

  % Form residuals
  Dt_sq = Dt * Dt;
  err_pos = (C_i' * ((r_j - r_i) - (v_i * Dt) + (0.5 * g * Dt_sq))) - dr;
  err_vel = (C_i' * ((v_j - v_i) + (g * Dt))) - dv;
  err_rot = (2 * quat_mul(quat_inv(dq), quat_mul(quat_inv(q_i), q_j)))(2:4);
  err_ba = ba_j - ba_i;
  err_bg = bg_j - bg_i;
  r = [err_pos; err_vel; err_rot; err_ba; err_bg];

  % Form jacobians
  jacs{1} = zeros(15, 6);  % w.r.t pose i
  jacs{2} = zeros(15, 9);  % w.r.t speed and biase i
  jacs{3} = zeros(15, 6);  % w.r.t pose j
  jacs{4} = zeros(15, 9);  % w.r.t speed and biase j

  % -- Jacobian w.r.t. pose i
  jacs{1}(1:3, 1:3) = -C_i';                                                       % dr w.r.t r_i
  jacs{1}(1:3, 4:6) = skew(C_i' * ((r_j - r_i) - (v_i * Dt) + (0.5 * g * Dt_sq))); % dr w.r.t C_i
  jacs{1}(4:6, 4:6) = skew(C_i' * ((v_j - v_i) + (g * Dt)));                       % dv w.r.t C_i
  jacs{1}(7:9, 4:6) = -Jr_inv(err_rot) * (C_j' * C_i);                             % dC w.r.t C_i

  % -- Jacobian w.r.t. speed and biases i
  jacs{2}(1:3, 1:3) = -C_i' * Dt;  % dr w.r.t v_i
  jacs{2}(1:3, 4:6) = -dr_dba;     % dr w.r.t ba
  jacs{2}(1:3, 7:9) = -dr_dbg;     % dr w.r.t bg

  jacs{2}(4:6, 1:3) = -C_i';       % dv w.r.t v_i
  jacs{2}(4:6, 4:6) = -dv_dba;     % dv w.r.t ba
  jacs{2}(4:6, 7:9) = -dv_dbg;     % dv w.r.t bg

  jacs{2}(7:9, 7:9) = -quat_left(C_j' * C_i * dC)(2:4, 2:4) * dq_dbg;     % dC w.r.t bg

  jacs{2}(10:12, 4:6) = -eye(3);   % dba w.r.t ba
  jacs{2}(13:15, 7:9) = -eye(3);   % dbg w.r.t bg

  % -- Jacobian w.r.t. pose j
  jacs{3}(1:3, 1:3) = C_i';                                                                       % dr w.r.t r_j
  jacs{3}(7:9, 4:6) = Jr_inv(err_rot);                                                           % dC w.r.t C_i
  % jacs{3}(7:9, 4:6) = quat_left(quat_mul(quat_inv(dq), quat_mul(quat_inv(q_i), q_j)))(2:4, 2:4);

  % -- Jacobian w.r.t. sb j
  jacs{4}(4:6, 1:3) = C_i';      % dv w.r.t v_j
  jacs{4}(10:12, 4:6) = eye(3);  % dv w.r.t ba_j
  jacs{4}(13:15, 7:9) = eye(3);  % dv w.r.t bg_j
endfunction
