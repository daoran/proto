#!/usr/bin/octave
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

function x_imu = imu_init()
  x_imu.p_WS = zeros(3, 1);
  x_imu.v_WS = zeros(3, 1);
  x_imu.C_WS = eye(3);
  x_imu.b_a = zeros(3, 1);
  x_imu.b_g = zeros(3, 1);
  x_imu.g = [0; 0; -9.81];
endfunction

function x_imu = imu_update(x_imu, a_B, w_B, dt)
  b_a = x_imu.b_a;
  b_g = x_imu.b_g;
  g = x_imu.g;
  n_a = zeros(3, 1);
  n_g = zeros(3, 1);

  C_WS = x_imu.C_WS;
  v_WS = x_imu.v_WS;
  p_WS = x_imu.p_WS;

  x_imu.C_WS = C_WS * so3_exp((w_B - b_g - n_g) * dt);
  x_imu.v_WS += (C_WS * (a_B - b_a - n_a) * dt) + (g * dt);
  x_imu.p_WS += (v_WS * dt) + (0.5 * C_WS * (a_B - b_a - n_a) * dt^2) + (0.5 * g * dt^2);
endfunction

% Initialize IMU state
x_imu = imu_init();
x_imu.p_WS = [0.0; 0.0; 0.0];
x_imu.v_WS = [0.0; 0.0; 0.0];
x_imu.C_WS = eye(3);

traj_pos = [x_imu.p_WS];
traj_vel = [x_imu.v_WS];
traj_att = [x_imu.C_WS];
t_prev = 0.0;

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
