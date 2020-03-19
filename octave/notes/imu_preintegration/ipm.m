#!/usr/bin/octave
addpath(genpath("."));
graphics_toolkit("fltk");

function C = so3_exp(phi)
  if (phi < 1e-3)
    C = eye(3) + skew(phi);
  else
    C = eye(3);
    C += (sin(norm(phi)) / norm(phi)) * skew(phi);
    C += ((1 - cos(norm(phi))) / norm(phi)^2) * skew(phi)^2;
  endif
endfunction

function x_imu = imu_state_init()
  x_imu.p_WS = zeros(3, 1);
  x_imu.v_WS = zeros(3, 1);
  x_imu.C_WS = eye(3);
  x_imu.b_a = zeros(3, 1);
  x_imu.b_g = zeros(3, 1);
  x_imu.g = [0; 0; -9.81];
endfunction

function x_imu = imu_update(x_imu, a_B, w_B, dt)
  g = x_imu.g;
  b_a = x_imu.b_a;
  b_g = x_imu.b_g;
  n_a = zeros(3, 1);
  n_g = zeros(3, 1);

  C_WS_i = x_imu.C_WS;
  v_WS_i = x_imu.v_WS;

  w_W = (w_B - b_g - n_g);
  a_W = (a_B - b_a - n_a);
  dt_sq = dt * dt;

  x_imu.C_WS *= so3_exp(w_W * dt);
  x_imu.v_WS += (C_WS_i * a_W * dt) + (g * dt);
  x_imu.p_WS += (v_WS_i * dt) + (0.5 * C_WS_i * a_W * dt_sq) + (0.5 * g * dt_sq);
endfunction

function imu_batch_integration(data)
  % Initialize IMU state
  x_imu = imu_state_init();
  x_imu.p_WS = data.oxts.p_G(:, 1);
  x_imu.v_WS = data.oxts.v_G(:, 1);
  x_imu.C_WS = euler321(data.oxts.rpy(:, 1));

  % Batch integrate IMU measurements
  traj_pos = [x_imu.p_WS];
  traj_vel = [x_imu.v_WS];
  traj_att = [rot2euler(x_imu.C_WS)];
  t_prev = data.oxts.time(1);
  for k = 2:length(data.oxts.time)
    % Calculate dt
    t = data.oxts.time(k);
    dt = t - t_prev;

    % Propagate IMU state
    a_B = data.oxts.a_B(:, k);
    w_B = data.oxts.w_B(:, k);
    x_imu = imu_update(x_imu, a_B, w_B, dt);

    % Keep track of t_prev
    traj_pos = [traj_pos, x_imu.p_WS];
    traj_vel = [traj_vel, x_imu.v_WS];
    traj_att = [traj_att, rot2euler(x_imu.C_WS)];
    t_prev = t;
  endfor

  % plot_imu(data);
  % plot_pos(data, traj_pos);
  plot_vel(data, traj_vel);
  % plot_att(data, traj_att);
endfunction

function x_imu = imu_preintegrate(x_imu, imu_ts, imu_a, imu_g)
  assert(length(imu_a) == length(imu_g));

  g = x_imu.g;
  b_a = x_imu.b_a;
  b_g = x_imu.b_g;
  n_a = zeros(3, 1);
  n_g = zeros(3, 1);

  dC = eye(3);
  dv = x_imu.v_WS;
  da = zeros(3, 1);

  dt_ij = imu_ts(end) - imu_ts(1);
  % for k = 1:(length(imu_a)-1)
    % dt = imu_ts(k + 1) - imu_ts(k);
  for k = 1:length(imu_a)
    dt = 0.1;
    dt_sq = dt * dt;

    w_k = imu_g(:, k);
    a_k = imu_a(:, k);

    dC *= so3_exp((w_k - b_g - n_g) * dt);
    dv += (dC * (a_k - b_a - n_a)) * dt;
    da += (dv * dt) + (0.5 * dC * (a_k - b_a - n_a) * dt_sq);
  endfor

  x_imu.C_WS *= dC;
  x_imu.v_WS += dv;
  x_imu.p_WS += da;
endfunction

function imu_relative_integration(data)
  % Initialize IMU state
  x_imu = imu_state_init();
  x_imu.p_WS = data.oxts.p_G(:, 1);
  x_imu.v_WS = data.oxts.v_G(:, 1);
  x_imu.C_WS = euler321(data.oxts.rpy(:, 1));

  % Relative integrate IMU measurements
  traj_time = [0.0];
  traj_pos = [x_imu.p_WS];
  traj_vel = [x_imu.v_WS];
  traj_att = [rot2euler(x_imu.C_WS)];
  t_prev = data.oxts.time(1);

  data.oxts.a_B(:, 1:10)

  km1 = 1;
  for k = 3:3:length(data.oxts.time)
    % Calculate dt
    t = data.oxts.time(k);
    dt = t - t_prev;

    % Propagate IMU state
    imu_ts = data.oxts.time(km1:k);
    imu_a = data.oxts.a_B(:, km1:k);
    imu_w = data.oxts.w_B(:, km1:k);
    x_imu = imu_preintegrate(x_imu, imu_ts, imu_a, imu_w);
    km1 = k + 1;

    % Keep track of t_prev
    traj_time = [traj_time, t];
    traj_pos = [traj_pos, x_imu.p_WS];
    traj_vel = [traj_vel, x_imu.v_WS];
    traj_att = [traj_att, rot2euler(x_imu.C_WS)];
    t_prev = t;

    % Velocity
    figure(2);
    hold on;
    % -- Velocity in x
    subplot(311);
    plot(traj_time, traj_vel(1, :), 'r-', 'linewidth', 2);
    title("Velocity in x");
    xlabel("Time [s]");
    ylabel("Velocity [m/s]");
    xlim([0.0, max(traj_time)]);
    % -- Velocity in y
    subplot(312);
    plot(traj_time, traj_vel(2, :), 'g-', 'linewidth', 2);
    title("Velocity in y");
    xlabel("Time [s]");
    ylabel("Velocity [m/s]");
    xlim([0.0, max(traj_time)]);
    % -- Velocity in z
    subplot(313);
    plot(traj_time, traj_vel(3, :), 'b-', 'linewidth', 2);
    title("Velocity in z");
    xlabel("Time [s]");
    ylabel("Velocity [m/s]");
    xlim([0.0, max(traj_time)]);
    ginput();
  endfor

  % Position
  figure(4);
  hold on;
  plot(traj_pos(1, :), traj_pos(2, :), 'r-', 'linewidth', 2)

  % Velocity
  figure(2);
  hold on;
  % -- Velocity in x
  subplot(311);
  plot(traj_time, traj_vel(1, :), 'r-', 'linewidth', 2);
  title("Velocity in x");
  xlabel("Time [s]");
  ylabel("Velocity [m/s]");
  xlim([0.0, max(traj_time)]);
  % -- Velocity in y
  subplot(312);
  plot(traj_time, traj_vel(2, :), 'g-', 'linewidth', 2);
  title("Velocity in y");
  xlabel("Time [s]");
  ylabel("Velocity [m/s]");
  xlim([0.0, max(traj_time)]);
  % -- Velocity in z
  subplot(313);
  plot(traj_time, traj_vel(3, :), 'b-', 'linewidth', 2);
  title("Velocity in z");
  xlabel("Time [s]");
  ylabel("Velocity [m/s]");
  xlim([0.0, max(traj_time)]);

  % Attitude
  figure(3);
  hold on;
  subplot(311);
  plot(traj_time, rad2deg(traj_att(1, :)), 'r-', 'linewidth', 2);
  title("Attitude in x");
  xlabel("Time [s]");
  ylabel("Attitude [deg]");
  xlim([0.0, max(traj_time)]);

  subplot(312);
  plot(traj_time, rad2deg(traj_att(2, :)), 'g-', 'linewidth', 2);
  title("Attitude in y");
  xlabel("Time [s]");
  ylabel("Attitude [deg]");
  xlim([0.0, max(traj_time)]);

  subplot(313);
  plot(traj_time, rad2deg(traj_att(3, :)), 'b-', 'linewidth', 2);
  title("Attitude in z");
  xlabel("Time [s]");
  ylabel("Attitude [deg]");
  xlim([0.0, max(traj_time)]);
endfunction


% Load KITTI Raw data sequence
data_path = "/data/kitti";
data_date = "2011_09_26";
data_seq = "0005";
data = load_kitti(data_path, data_date, data_seq);
imu_batch_integration(data);
imu_relative_integration(data);
ginput();
