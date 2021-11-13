#!/usr/bin/octave
addpath(genpath("proto"));
addpath(genpath("notes"));
graphics_toolkit("fltk");

function plot_imu(sim_data)
  % Plot positions and velocities
  figure();
  subplot(311);
  hold on;
  plot(sim_data.imu_time, sim_data.imu_pos(1, :), 'r-', 'linewidth', 2.0);
  plot(sim_data.imu_time, sim_data.imu_pos(2, :), 'g-', 'linewidth', 2.0);
  plot(sim_data.imu_time, sim_data.imu_pos(3, :), 'b-', 'linewidth', 2.0);
  xlabel("Time [s]");
  ylabel("Displacement [m]");
  legend('x', 'y', 'z');

  subplot(312);
  hold on;
  plot(sim_data.imu_time, sim_data.imu_vel(1, :), 'r-', 'linewidth', 2.0);
  plot(sim_data.imu_time, sim_data.imu_vel(2, :), 'g-', 'linewidth', 2.0);
  plot(sim_data.imu_time, sim_data.imu_vel(3, :), 'b-', 'linewidth', 2.0);
  xlabel("Time [s]");
  ylabel("Velocity [ms^{-1}]");
  legend('x', 'y', 'z');

  subplot(313);
  hold on;
  plot(sim_data.imu_pos(1, :), sim_data.imu_pos(2, :), 'r-', 'linewidth', 2.0);
  xlabel("Displacement [m]");
  ylabel("Displacement [m]");
  axis('equal');

  % Plot accelerometer and gyroscope measurements
  figure();
  subplot(211);
  hold on;
  plot(sim_data.imu_time, sim_data.imu_acc(1, :), 'r-', 'linewidth', 2.0);
  plot(sim_data.imu_time, sim_data.imu_acc(2, :), 'g-', 'linewidth', 2.0);
  plot(sim_data.imu_time, sim_data.imu_acc(3, :), 'b-', 'linewidth', 2.0);
  xlabel("Time [s]");
  ylabel("Acceleration [ms^{-2}]");
  legend('x', 'y', 'z');

  subplot(212);
  hold on;
  plot(sim_data.imu_time, sim_data.imu_gyr(1, :), 'r-', 'linewidth', 2.0);
  plot(sim_data.imu_time, sim_data.imu_gyr(2, :), 'g-', 'linewidth', 2.0);
  plot(sim_data.imu_time, sim_data.imu_gyr(3, :), 'b-', 'linewidth', 2.0);
  xlabel("Time [s]");
  ylabel("Angular Velocity [rad s^{-1}]");
  legend('x', 'y', 'z');

  % Plot scene
  figure();
  hold on;
  nb_poses = length(sim_data.imu_poses);
  interval = int32(nb_poses / 10);
  for k = 1:interval:nb_poses
    T_WS = sim_data.imu_poses{k};
    draw_frame(T_WS, 0.1);
  endfor
  axis('equal');
  xlabel('x [m]');
  ylabel('y [m]');
  zlabel('z [m]');
  view(3);

  ginput()
endfunction

function msckf = msckf_init()
  msckf = {};

  % Settings
  msckf.window_size = 5;
  % -- IMU
  msckf.imu_rate = 200.0;
  % -- Process noise
  msckf.sigma_na = 0.01;
  msckf.sigma_nw = 0.01;
  msckf.sigma_ba = 0.01;
  msckf.sigma_bg = 0.01;

  % Stats
  msckf.prev_ts = 0.0;

  % State Vector [r, v, q, ba, bg]
  msckf.state.r_WS = zeros(3, 1);
  msckf.state.v_WS = zeros(3, 1);
  msckf.state.C_WS = eye(3);
  msckf.state.ba   = zeros(3, 1);
  msckf.state.bg   = zeros(3, 1);
  msckf.state.g    = [0; 0; 9.81];

  % State Jacobian and covariance
  msckf.F = eye(15, 15);
  msckf.P = eye(15, 15);

  % Process noise matrix Q
  msckf.Q = zeros(12, 12);
  msckf.Q(1:3, 1:3)     = msckf.sigma_na**2 * eye(3);
  msckf.Q(4:6, 4:6)     = msckf.sigma_nw**2 * eye(3);
  msckf.Q(7:9, 7:9)     = msckf.sigma_ba**2 * eye(3);
  msckf.Q(10:12, 10:12) = msckf.sigma_bg**2 * eye(3);

  % Features
  features = {};
endfunction

function msckf = msckf_prediction_update(msckf, ts, a_m, w_m)
  % Setup
  v_WS_k = msckf.state.v_WS;
  C_WS_k = msckf.state.C_WS;
  q_WS_k = rot2quat(C_WS_k);
  ba     = msckf.state.ba;
  bg     = msckf.state.bg;
  g      = msckf.state.g;
  dt     = 1.0 / msckf.imu_rate;

  % Prediction update via Runge-Kutta 4th Order
  a = a_m - ba;
  w = w_m - bg;
  % -- Integrate orientation at time k + dt (kpdt: k plus dt)
  q_WS_kpdt = quat_integrate(q_WS_k, w, dt);
  C_WS_kpdt = quat2rot(q_WS_kpdt);
  % -- Integrate orientation at time k + dt / 2 (kphdt: k plus half dt)
  q_WS_kphdt = quat_integrate(q_WS_k, w, dt / 2);
  C_WS_kphdt = quat2rot(q_WS_kphdt);
  % -- k1 = f(tn, yn)
  k1_v_dot = C_WS_k * a - g;
  k1_p_dot = v_WS_k;
  % -- k2 = f(tn + dt / 2, yn + k1 * dt / 2)
  k2_v_dot = C_WS_kphdt * a - g;
  k2_p_dot = v_WS_k + k1_v_dot * dt / 2;
  % -- k3 = f(tn + dt / 2, yn + k2 * dt / 2)
  k3_v_dot = C_WS_kphdt * a - g;
  k3_p_dot = v_WS_k + k2_v_dot * dt / 2;
  % -- k4 = f(tn + dt, tn + k3 * dt)
  k4_v_dot = C_WS_kpdt * a - g;
  k4_p_dot = v_WS_k + k3_v_dot * dt;
  % -- Update predicted state
  msckf.state.r_WS += dt / 6 * (k1_p_dot + 2 * k2_p_dot + 2 * k3_p_dot + k4_p_dot);
  msckf.state.v_WS += dt / 6 * (k1_v_dot + 2 * k2_v_dot + 2 * k3_v_dot + k4_v_dot);
  msckf.state.C_WS = C_WS_kpdt;

  % Jacobian of process model w.r.t. error vector - F
  F = zeros(15, 15);
  F(1:3, 4:6) = eye(3);
  F(4:6, 7:9) = -msckf.state.C_WS * skew(a);
  F(4:6, 10:12) = -msckf.state.C_WS;
  F(7:9, 7:9) = -skew(w);
  F(7:9, 13:15) = -eye(3);

  % Jacobian of process model w.r.t. impulse vector - G
  G = zeros(15, 12);
  G(4:6, 1:3) = -msckf.state.C_WS;
  G(7:9, 4:6) = -eye(3);
  G(10:12, 7:9) = eye(3);
  G(13:15, 10:12) = eye(3);

  % State covariance matrix P
  G_dt = G * dt;
  I_F_dt = eye(15) + F * dt;
  msckf.F = I_F_dt * msckf.F;
  msckf.P = I_F_dt * msckf.P * I_F_dt' + G_dt * msckf.Q * G_dt';
  msckf.prev_ts = ts;
endfunction

function msckf = msckf_augment_state(msckf)
  T_WS = tf(msckf.C_WS, msckf.r_WS);
  T_SC = tf(msckf.C_WS, msckf.r_WS);

  r_SC = tf_trans(T_SC);
  C_SC = tf_rot(T_SC);
  C_WS = tf_rot(T_WS);

  J = zeros(6, 15 + msckf.window_size);
  J(1:3, 1:3) = C_SC;
  J(4:6, 1:3) = skew(C_WS * r_SC);

  P_aug = [eye(6 * msckf.window_size + 15); J];
  msckf.P = P_aug * msckf.P * P_aug';
endfunction

function [r, H_x] = msckf_feature_jacobian(msckf, ts, feature_ids, keypoints)
  % Get camera pose in world frame
  T_WS = tf(msckf.C_WS, msckf.r_WS);
  T_SC = tf(msckf.C_SC, msckf.r_SC);
  T_WC = T_WS * T_SC;

  % Transform feature position to image point z_hat
  p_W = msckf.features{feature_id};
  p_C = inv(T_WC) * p_W;
  C_WC = tf_rot(T_WC);
  z_hat = [p_C(1) / p_C(3); p_C(2) / p_C(3)];

  % Form residual
  r = z - z_hat;

  % Form jacobians
  J_i = 1.0 / p_C(3) * [1.0, 0.0, -p_C(1) / p_C(3);
                        0.0, 1.0, -p_C(2) / p_C(3)];
  H_x = [J_i * skew(p_C), -J_i * skew(C_WC)];
  H_f = J_i * skew(C_WC);

  % Project residuals and feature Jacobian to Null Space of state Jacobian
  [U, S, V] = svd(H_f);
  A = U(:, 1:(2 * nb_cam_states));
  r = A' * H_f;
  H_x = A' * H_x;
endfunction

function [r, H_x] = msckf_measurement_jacobian(msckf, ts, feature_ids, keypoints)
  % Iterate over all features currently tracking and form a measurement jacobian
  H_x = []; % State jacobian
  H_f = []; % Feature jacobian

  for i = 1:length(feature_ids)
    feature_id = feature_ids(i);
    z = keypoints{i};
  endfor

endfunction

function msckf = msckf_measurement_update(msckf, ts, feature_ids, keypoints)
  msckf = msckf_augment_state(msckf);

  % Marginalize out feature jacobians and residuals via QR decomposition
  H = [];
  r = [];
  [Q, R] = qr(H);
  H_thin = (Q' * H)(1:(15 + 6 * nb_cam_states), 1:end);
  r_thin = (Q' * r)(1:(15 + 6 * nb_cam_states));

  % Calculate update
  % K = P * T' * inv(T * P * T' + R);
  K = msckf.P * H_thin' * inv(H_thin * msckf.P * H_thin' + msckf.R);
  % K = (H_thin * msckf.P * H_thin' + msckf.R) \ (msckf.P * H_thin');
  dx = K * r_thin;

  % Update state-vector


  % Update state-covariance
  I_KH = (eye() - K * H_thin);
  msckf.P = I_KH * msckf.P * I_KH' + K * msckf.R * K';
endfunction

% Kalman Filter
% -- Predict
% x = F * x + B * u;
% P = F * P * F' + G * Q * G';
% -- Update
% y = z - H * x
% K = P * T' * inv(T * P * T' + R);
% x = x + K * y;
% P = (eye() - K * T) * P * (eye() - K * T)' + K * R * K';

% Simulate imu data
save_path = "/tmp/msckf.data";
if length(glob(save_path)) == 0
  circle_r = 2.0;
  circle_velocity = 1.0;
  sim_data = sim_vio(circle_r, circle_velocity)
  save("-binary", save_path, "sim_data");
else
  load(save_path);
endif

msckf = msckf_init();
msckf.r_WS = tf_trans(sim_data.imu_poses{1});
msckf.v_WS = sim_data.imu_vel(:, 1);
msckf.C_WS = tf_rot(sim_data.imu_poses{1});

% Loop through data
enable_plot_positions = 0;
f = figure();
pos_est = [];
pos_gnd = [];

printf("Loop through time\n");
for k = 2:length(sim_data.timeline)
  printf(".");
  event = sim_data.timeline(k);
  t = event.time;

  if event.has_imu_data
    % Prediction update
    ts = sim_data.imu_time(k);
    acc = sim_data.imu_acc(:, k);
    gyr = sim_data.imu_gyr(:, k);
    msckf = msckf_prediction_update(msckf, ts, acc, gyr);
  endif

  if event.has_cam_data
    % Measurement update
    feature_ids = event.cam_p_data;
    keypoints = event.cam_z_data;
    feature_ids
    msckf = msckf_measurement_update(msckf, ts, feature_ids, keypoints);
  endif

  % Plot position
  if enable_plot_positions
    pos_est = [pos_est, msckf.state.r_WS];
    pos_gnd = [pos_est, sim_data.imu_pos(:, k)];
    plot(pos_gnd(1, :), pos_gnd(2, :), 'r-');
    plot(pos_est(1, :), pos_est(2, :), 'b-');
    xlabel("x [m]");
    ylabel("y [m]");
    axis('equal');
    pause(0.001);
  endif
endfor
