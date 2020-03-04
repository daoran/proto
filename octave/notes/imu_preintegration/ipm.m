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

  C_WS = x_imu.C_WS;
  v_WS = x_imu.v_WS;
  p_WS = x_imu.p_WS;

  x_imu.C_WS *= so3_exp((w_B - b_g - n_g) * dt);
  x_imu.v_WS += (C_WS * (a_B - b_a - n_a) * dt) + (g * dt);
  x_imu.p_WS += (v_WS * dt) + (0.5 * C_WS * (a_B - b_a - n_a) * dt^2) + (0.5 * g * dt^2);
endfunction

function x_imu = imu_preintegrate(x_imu, a_B, w_B, dt)
  assert(length(a_B) == length(w_B));

  g = x_imu.g;
  b_a = x_imu.b_a;
  b_g = x_imu.b_g;
  n_a = zeros(3, 1);
  n_g = zeros(3, 1);

  dC_ik = x_imu.C_WS;
  dv_ik = x_imu.v_WS;
  dp_ik = x_imu.p_WS;

  for k = 1:length(a_B)
    dC_ik *= so3_exp((w_B(:, k) - b_g) * dt);
    dv_ik += dC_ik * (a_B(:, k) - b_a) * dt;
    dp_ik += 1.5 * dC_ik * (a_B(:, k) - b_a) * dt**2;
  endfor

  x_imu.C_WS = dC_ik;
  x_imu.v_WS = dv_ik;
  x_imu.p_WS = dp_ik;
endfunction

% Load KITTI Raw data sequence
data_path = "/data/kitti";
data_date = "2011_09_26";
data_seq = "0005";
data = load_kitti(data_path, data_date, data_seq);

% Initialize IMU state
x_imu = imu_state_init();
x_imu.p_WS = data.oxts.p_G(:, 1);
x_imu.v_WS = data.oxts.v_G(:, 1);
x_imu.C_WS = euler321(data.oxts.rpy(:, 1));

traj_pos = [x_imu.p_WS];
traj_vel = [x_imu.v_WS];
traj_att = [rot2euler(x_imu.C_WS)];
t_prev = data.oxts.time(1);
% for k = 2:length(data.oxts.time)
%   % Calculate dt
%   t = data.oxts.time(k);
%   dt = t - t_prev;
%
%   % Propagate IMU state
%   a_B = data.oxts.a_B(:, k);
%   w_B = data.oxts.w_B(:, k);
%   x_imu = imu_update(x_imu, a_B, w_B, dt);
%
%   % Keep track of t_prev
%   traj_pos = [traj_pos, x_imu.p_WS];
%   traj_vel = [traj_vel, x_imu.v_WS];
%   traj_att = [traj_att, rot2euler(x_imu.C_WS)];
%   t_prev = t;
% endfor

km1 = 1;
for k = 10:10:length(data.oxts.time)
  % Calculate dt
  t = data.oxts.time(k);
  dt = t - t_prev;

  % Propagate IMU state
  a_B = data.oxts.a_B(:, km1:k);
  w_B = data.oxts.w_B(:, km1:k);
  x_imu = imu_preintegrate(x_imu, a_B, w_B, dt);
  km1 = k;

  % Keep track of t_prev
  traj_pos = [traj_pos, x_imu.p_WS];
  traj_vel = [traj_vel, x_imu.v_WS];
  traj_att = [traj_att, rot2euler(x_imu.C_WS)];
  t_prev = t;
endfor

% plot_imu(data);
plot_pos(data, traj_pos);
plot_vel(data, traj_vel);
plot_att(data, traj_att);
ginput();
