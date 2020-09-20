#!/usr/bin/octave
addpath(genpath("proto"));
pkg load "statistics";

graphics_toolkit("fltk");

function traj = sim_traj()
  % Settings
  imu_rate = 400.0;
  t_end = 10.0;
  dt = 1 / imu_rate;
  circle_r = 10.0;
  circle_cx = 0.0;
  circle_cy = 0.0;
  nb_features = 1000;

  % Create features
  origin = [circle_cx, circle_cy, 0.0];
  dim = [circle_r * 2, circle_r * 2, circle_r * 2];
  features = create_3d_features_perimeter(origin, dim, nb_features);

  sim_time = linspace(0.0, t_end, t_end * imu_rate);
  circle_theta = linspace(-pi, pi, t_end * imu_rate);

  poses = {};
  pos = {};
  vel = {};
  acc = {};
  ang_vel = {};
  k = 1;
  for theta = circle_theta
    % Pose
    % -- Position
    x = circle_r * cos(theta) + circle_cx;
    y = circle_r * sin(theta) + circle_cy;
    z = 0.0;
    r = [x; y; z];
    % -- Orientation
    yaw = theta + pi / 2.0;
    C = euler321([0.0, 0.0, yaw]);
    T = tf(C, r);

    % Velocity
    vx = -circle_r * sin(theta) * ((2 * pi) / t_end);
    vy = circle_r * cos(theta) * ((2 * pi) / t_end);
    vz = 0.0;

    % Acceleration
    ax = -circle_r * cos(theta);
    ay = -circle_r * sin(theta);
    az = 0.0;

    % Angular Velocity
    wx = 0.0;
    wy = 0.0;
    wz = norm([vx, vy]) / circle_r;

    % Update
    poses{k} = T;
    pos{k} = [x; y; z];
    vel{k} = [vx; vy; vz];
    acc{k} = [ax; ay; az];
    ang_vel{k} = [wx; wy; wz];
    k += 1;
  end

  % Create trajectory structure
  traj = {};
  traj.time = sim_time;
  traj.dt = 1.0 / imu_rate;
  traj.T_WS_W = poses;
  traj.r_WS_W = pos;
  traj.v_WS_W = vel;
  traj.a_WS_W = acc;
  traj.w_WS_W = ang_vel;

  % Plot data
  plot_data = 0;
  if plot_data
    figure();
    hold on;
    for k = 1:100:length(traj.T_WS_W)
      T_WS_W = traj.T_WS_W{k};
      draw_frame(T_WS_W);
    end
    plot3(features(:, 1), features(:, 2), features(:, 3), 'r.');
    xlabel("x [m]");
    ylabel("y [m]");
    plot(acc(2, :), 'g-', 'linewidth', 2);
    title("Position");

    figure();
    hold on;
    vel = cell2mat(vel);
    plot(vel(1, :), 'r-', 'linewidth', 2);
    plot(vel(2, :), 'g-', 'linewidth', 2);
    title("Velocity");

    figure();
    hold on;
    acc = cell2mat(acc);
    plot(acc(1, :), 'r-', 'linewidth', 2);
    plot(acc(2, :), 'g-', 'linewidth', 2);
    title("Acceleration");

    ginput()
  endif
endfunction

function imu = sim_imu_init()
  imu = {};
  imu.rate = 0.0;             % IMU rate [Hz]
  imu.tau_g = 3600;          % Reversion time constant for accel [s]
  imu.tau_a = 3600;          % Reversion time constant for gyro [s]
  imu.sigma_g_c = 12.0e-4;   % Gyro noise density [rad/s/sqrt(Hz)]
  imu.sigma_a_c = 8.0e-3;    % Accel noise density [m/s^s/sqrt(Hz)]
  imu.sigma_gw_c = 4.0e-6;   % Gyro drift noise density [rad/s^s/sqrt(Hz)]
  imu.sigma_aw_c = 4.0e-5;   % Accel drift noise density [m/s^2/sqrt(Hz)]
  imu.g = 9.81;               % Gravity vector [ms-2]

  imu.b_g = zeros(3, 1);
  imu.b_a = zeros(3, 1);
  imu.w_WS_S = zeros(3, 1);
  imu.a_WS_S = zeros(3, 1);
  imu.ts_prev = 0;
endfunction

function imu = sim_imu_measurements(imu, T_WS_W, w_WS_W, a_WS_W, dt)
  mu = [0.0; 0.0; 0.0];
  sigma = 1.0;

  % Propagate biases (slow moving signal)
  w_g = mvnrnd(mu, sigma); % Gyro white noise
  w_a = mvnrnd(mu, sigma); % Accel white noise
  imu.b_g += -imu.b_g / imu.tau_g * dt + w_g * imu.sigma_gw_c * sqrt(dt);
  imu.b_a += -imu.b_a / imu.tau_a * dt + w_a * imu.sigma_aw_c * sqrt(dt);

  % Compute gyro measurement
  C_WS = tf_rot(T_WS_W);
  C_SW = C_WS';
  % w_g = mvnrnd(mu, sigma); % Gyro white noise
  % imu.w_WS_S = C_SW * w_WS_W + imu.b_g + w_g * imu.sigma_g_c * sqrt(dt);
  imu.w_WS_S = C_SW * w_WS_W;

  % Compute accel measurement
  g = [0.0; 0.0; -imu.g]; % Gravity vector
  w_a = mvnrnd(mu, sigma);   % Accel white noise
  % imu.a_WS_S = C_SW * (a_WS_W - g) + imu.b_a + w_a * imu.sigma_a_c * sqrt(dt);
  imu.a_WS_S = C_SW * (a_WS_W - g);
endfunction

function C = so3_exp(phi)
  if (phi < 1e-3)
    C = eye(3) + skew(phi);
  else
    C = eye(3);
    C += (sin(norm(phi)) / norm(phi)) * skew(phi);
    C += ((1 - cos(norm(phi))) / norm(phi)^2) * skew(phi)^2;
  endif
endfunction

function x_imu = imu_init()
  x_imu.r_WS = zeros(3, 1);
  x_imu.v_WS = zeros(3, 1);
  x_imu.C_WS = eye(3);
  x_imu.b_a = zeros(3, 1);
  x_imu.b_g = zeros(3, 1);
  x_imu.g = [0; 0; -9.81];
endfunction

function x_imu = imu_update(x_imu, w_B, a_B, dt)
  b_a = x_imu.b_a;
  b_g = x_imu.b_g;
  g = x_imu.g;
  n_a = zeros(3, 1);
  n_g = zeros(3, 1);

  C_WS = x_imu.C_WS;
  v_WS = x_imu.v_WS;
  r_WS = x_imu.r_WS;

  x_imu.C_WS = C_WS * so3_exp((w_B - b_g - n_g) * dt);
  x_imu.r_WS += (v_WS * dt) + (0.5 * C_WS * (a_B - b_a - n_a) * dt**2) + (0.5 * g * dt**2);
  x_imu.v_WS += (C_WS * (a_B - b_a - n_a) * dt) + (g * dt);
  a_B
endfunction

% Simulate trajectory
traj = sim_traj();
sim_imu = sim_imu_init();
gt_pos = cell2mat(traj.r_WS_W);
gt_vel = cell2mat(traj.v_WS_W);
gt_acc = cell2mat(traj.a_WS_W);

% Initialize IMU state
x_imu = imu_init();
x_imu.r_WS = tf_trans(traj.T_WS_W{1});
x_imu.v_WS = traj.v_WS_W{1};
x_imu.C_WS = tf_rot(traj.T_WS_W{1});

r_WS = tf_trans(traj.T_WS_W{1})
v_WS = traj.v_WS_W{1}

imu_pos = [];
imu_vel = [];
imu_att = [];
imu_acc_m = [];
imu_gyro_m = [];

for k = 1:length(traj.time)
  T_WS_W = traj.T_WS_W{k};
  w_WS_W = traj.w_WS_W{k};
  a_WS_W = traj.a_WS_W{k};

  % sim_imu = sim_imu_measurements(sim_imu, T_WS_W, w_WS_W, a_WS_W, traj.dt);
  % x_imu = imu_update(x_imu, sim_imu.w_WS_S, sim_imu.a_WS_S, traj.dt);

  dt = traj.dt;
  g = [0.0; 0.0; -9.81];
  C_WS = tf_rot(T_WS_W);
  % a_B = C_WS' * (a_WS_W - g);
  % r_WS += (v_WS * dt) + (0.5 * C_WS * a_B * dt**2) + (0.5 * g * dt**2);
  % v_WS += (C_WS * a_B * dt) + (g * dt);

  a_WS = a_WS_W;
  v_WS = traj.v_WS_W{k};
  r_WS += v_WS * dt + (0.5 * a_WS_W * dt^2);

  % r_WS += v_WS * dt + (0.5 * a_WS_W * dt^2);
  % v_WS += a_WS_W * dt;

  % imu_pos = [imu_pos, x_imu.r_WS];
  % imu_vel = [imu_vel, x_imu.v_WS];
  imu_pos = [imu_pos, r_WS];
  imu_vel = [imu_vel, v_WS];
  imu_acc = [imu_vel, a_WS];
  % imu_att = [imu_att, rot2euler(x_imu.C_WS)];
  % imu_acc_m = [imu_acc_m, sim_imu.a_WS_S];
  % imu_gyro_m = [imu_gyro_m, sim_imu.w_WS_S];
endfor

% figure();
% att = [];
% for k = 1:length(traj.T_WS_W)
%   T_WS_W = traj.T_WS_W{k};
%   C_WS = tf_rot(T_WS_W);
%   att = [att, rot2euler(C_WS)];
% endfor
% plot(att(1, :), 'r-', 'linewidth', 2.0);
% plot(att(2, :), 'g-', 'linewidth', 2.0);
% plot(att(3, :), 'b-', 'linewidth', 2.0);

figure(1);
hold on;
plot(gt_pos(1, :), gt_pos(2, :), 'r-', 'linewidth', 2.0);
plot(imu_pos(1, :), imu_pos(2, :), 'b-', 'linewidth', 2.0);
legend("Ground Truth", "IMU model");

figure(2);
hold on;
plot(gt_vel(1, :), 'r-', 'linewidth', 2.0);
plot(imu_vel(1, :), 'r--', 'linewidth', 2.0);

plot(gt_vel(2, :), 'g-', 'linewidth', 2.0);
plot(imu_vel(2, :), 'g--', 'linewidth', 2.0);

plot(gt_vel(3, :), 'b-', 'linewidth', 2.0);
plot(imu_vel(3, :), 'b--', 'linewidth', 2.0);

figure(3);
hold on;
plot(gt_acc(1, :), 'r-', 'linewidth', 2.0);
plot(imu_acc(1, :), 'r--', 'linewidth', 2.0);

plot(gt_acc(2, :), 'g-', 'linewidth', 2.0);
plot(imu_acc(2, :), 'g--', 'linewidth', 2.0);

plot(gt_acc(3, :), 'b-', 'linewidth', 2.0);
plot(imu_acc(3, :), 'b--', 'linewidth', 2.0);

% figure(2);
% limit = 1000;
% subplot(211)
% hold on;
% plot(gt_vel(1, 1:limit), 'r-', 'linewidth', 2.0);
% plot(imu_vel(1, 1:limit), 'r--', 'linewidth', 2.0);
% legend("Ground Truth", "IMU model");
% subplot(212)
% hold on;
% plot(gt_vel(2, 1:limit), 'g-', 'linewidth', 2.0);
% plot(imu_vel(2, 1:limit), 'g--', 'linewidth', 2.0);
% legend("Ground Truth", "IMU model");

% figure();
% limit = 100;
% % -- Accelerometer
% subplot(211);
% hold on;
% plot(imu_acc_m(1, 1:limit), 'r-', 'linewidth', 2.0);
% plot(imu_acc_m(2, 1:limit), 'g-', 'linewidth', 2.0);
% plot(imu_acc_m(3, 1:limit), 'b-', 'linewidth', 2.0);
% title("Accelerometer");
% % -- Gyroscope
% subplot(212);
% hold on;
% plot(imu_gyro_m(1, 1:limit), 'r-', 'linewidth', 2.0);
% plot(imu_gyro_m(2, 1:limit), 'g-', 'linewidth', 2.0);
% plot(imu_gyro_m(3, 1:limit), 'b-', 'linewidth', 2.0);
% title("Gyroscope");

ginput();
