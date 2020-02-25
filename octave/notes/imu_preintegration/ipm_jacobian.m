#!/usr/bin/octave
addpath(genpath("."));

% Load KITTI Raw data sequence
data_path = "/data/kitti";
data_date = "2011_09_26";
data_seq = "0005";
data = load_kitti(data_path, data_date, data_seq);

dt = data.oxts.time(2) - data.oxts.time(1);
g = [0.0; 0.0; -10.0];

C_WS = euler321(data.oxts.rpy(:, 1));
b_a = zeros(3, 1);
n_a = zeros(3, 1);

a_Bk = data.oxts.a_B(:, 1);
w_Bk = data.oxts.w_B(:, 1);
a_Bkp1 = data.oxts.a_B(:, 2);
w_Bkp1 = data.oxts.w_B(:, 2);
a_true = 0.5 * (a_Bk + a_Bk);
w_true = 0.5 * (w_Bk + w_Bkp1);

dp_k = 0.0;
dv_k = 0.0;
dq_k = [1.0; 0.0; 0.0; 0.0];

% delta position
dp_kp1 = dp_k + dv_k * dt + a_true * dt^2;
% delta velocity
dv_kp1 = dv_k + a_true * dt;
% delta orientation
dq_kp1 = quatmul(
  dq_k,
  [1.0;
  w_true(1) * dt / 2.0;
  w_true(2) * dt / 2.0;
  w_true(3) * dt / 2.0]
);

% (C_WS * (a_Bk - b_a - n_a) * dt);

dalpha_dp = eye(3);
dalpha_dq = -0.25 * quat2rot(dq_k) * skew(a_Bk) * dt^2 + -0.25 * quat2rot(dq_kp1) * dt^2;
dalpha_dv = eye(3) * dt;

function check_Jdp(q_k, q_kp1)
  % for i = 1:3
  %   T_SC0_diff = perturb_trans(T_SC0, step_size, i);
  %   e_prime = residual(T_WS, T_SC0_diff, T_WF, p_F);
  %   fdiff(1:2, i) = (e_prime - e) / step_size;
  % endfor
endfunction
