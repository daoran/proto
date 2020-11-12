addpath(genpath("proto"));
graphics_toolkit("fltk");

step_size = 1e-8;
threshold = 1e-5;

C_ij = euler321(deg2rad([0.1, 0.2, 0.3]));
C_si_hat = euler321(deg2rad([0.01, 0.01, 0.01]));
C_sj_hat = euler321(deg2rad([0.1, 0.2, 0.3]));
C_ij_hat = C_si_hat' * C_sj_hat;

function dtheta = residual(C_ij, C_ij_hat)
  dC = C_ij * inv(C_ij_hat);
  dq = rot2quat(dC);
  dtheta = 2 * dq(2:4);
endfunction

function dtheta = residual2(C_ij, C_si, C_sj)
  C_ij_hat = C_si' * C_sj;
  dC = C_ij * inv(C_ij_hat);
  dq = rot2quat(dC);

  dtheta = 2 * dq(2:4);
endfunction

function Q = quat_plus_xyz(q)
  qw = q(1);
  qx = q(2);
  qy = q(3);
  qz = q(4);

  % Q = [qw, -qz, qy;
  %      qz, qw, -qx;
  %      -qy, qx, qw];
  Q = eye(3) * qw + skew([qx, qy, qz]);
endfunction

fdiff = zeros(3, 3);
r = residual(C_ij, C_ij_hat);
for i = 1:3
  % C_ij_hat_diff = perturb_rot(C_ij_hat, step_size, i);

  C_si_hat_diff = perturb_rot(C_si_hat, step_size, i);
  C_ij_hat_diff = C_si_hat_diff' * C_sj_hat;

  % C_sj_hat_diff = perturb_rot(C_sj_hat, step_size, i);
  % C_ij_hat_diff = C_si_hat' * C_sj_hat_diff;

  r_prime = residual(C_ij, C_ij_hat_diff);
  fdiff(1:3, i) = (r_prime - r) / step_size;
endfor
fdiff

% dq = -rot2quat(C_ij * C_ij_hat');
% dtheta = 2 * dq(2:4);
% jac = quat_plus_xyz(dq)

% q_ij = rot2quat(C_ij);
% q_si_hat = rot2quat(C_si_hat);
% q_sj_hat = rot2quat(C_sj_hat);
% q_is_hat = quat_inv(q_si_hat);
% q_js_hat = quat_inv(q_sj_hat);
% q_ij_hat = quat_mul(q_is_hat, q_sj_hat);
% q_ji_hat = quat_inv(q_ij_hat);
%
% dq = quat_mul(q_ij, -q_ji_hat);
% dtheta = 2 * dq(2:4);
% jac = quat_plus_xyz(dq)

% q_ij = rot2quat(C_ij);
% q_si_hat = rot2quat(C_si_hat);
% q_sj_hat = rot2quat(C_sj_hat);
% q_is_hat = quat_inv(q_si_hat);
% q_js_hat = quat_inv(q_sj_hat);
% q_ij_hat = quat_mul(q_is_hat, q_sj_hat);
% q_ji_hat = quat_inv(q_ij_hat);
% q_js_hat = quat_mul(q_ji_hat, q_is_hat);
%
% dq = quat_mul(q_ij, q_js_hat);
% dtheta = 2 * dq(2:4);
% jac = quat_plus_xyz(dq)


q_ij = rot2quat(C_ij);
q_si_hat = rot2quat(C_si_hat);
q_sj_hat = rot2quat(C_sj_hat);
q_is_hat = quat_inv(q_si_hat);
q_js_hat = quat_inv(q_sj_hat);
q_ij_hat = quat_mul(q_is_hat, q_sj_hat);
q_ji_hat = quat_inv(q_ij_hat);
q_js_hat = quat_mul(q_ji_hat, q_is_hat);

dq = quat_mul(quat_mul(q_ij, -quat_inv(q_ij_hat)), quat_inv(q_si_hat));
dtheta = 2 * dq(2:4);
jac = quat_plus_xyz(dq)



% function check_jac_wrt_Cji(C_ij, C_si, C_sj, step_size, threshold)
%   fdiff = zeros(3, 3);
%   C_ji = C_sj' * C_si;
%
%   q_ij = rot2quat(C_ij);
%   q_ji = rot2quat(C_ji);
%   dquat = quat_mul(q_ji, q_ij);
%   % C_ii = C_ij * C_ji;
%   % C_jj = C_ij * C_ji;
%
%   r = residual(C_ij, C_ji);
%   for i = 1:3
%     C_ji_diff = perturb_rot(C_ji, step_size, i);
%     r_prime = residual(C_ij, C_ji_diff);
%     fdiff(1:3, i) = (r_prime - r) / step_size;
%   endfor
%
%   jac_name = "dCii_dCji";
%   fdiff
%   % jac = quat_plus_xyz(rot2quat(C_ii))
%   jac = quat_plus_xyz(dquat)
%   retval = check_jacobian(jac_name, fdiff, jac, threshold, print=true);
% endfunction

% function check_jac_wrt_Csi(C_ij, C_si, C_sj, step_size, threshold)
%   fdiff = zeros(3, 3);
%   r = residual(C_ij, C_si, C_sj);
%   for i = 1:3
%     C_si_diff = perturb_rot(C_si, step_size, i);
%     r_prime = residual(C_ij, C_si_diff, C_sj);
%     fdiff(1:3, i) = (r_prime - r) / step_size;
%   endfor
%
%   jac_name = "dCii_dCsi";
%   fdiff
%   jac = quat_plus_xyz(rot2quat(C_ij * -C_si'))
%   retval = check_jacobian(jac_name, fdiff, jac, threshold, print=true);
% endfunction
%
% function check_jac_wrt_Csj(C_ij, C_si, C_sj, step_size, threshold)
%   fdiff = zeros(3, 3);
%   r = residual(C_ij, C_si, C_sj);
%   for i = 1:3
%     C_sj_diff = perturb_rot(C_sj, step_size, i);
%     r_prime = residual(C_ij, C_si, C_sj_diff);
%     fdiff(1:3, i) = (r_prime - r) / step_size;
%   endfor
%
%   jac_name = "dCii_dCsj";
%   fdiff
%   jac = -quat_plus_xyz(rot2quat(C_ij))
%   retval = check_jacobian(jac_name, fdiff, jac, threshold, print=true);
% endfunction

% check_jac_wrt_Cji(C_ij, C_si, C_sj, step_size, threshold)
% check_jac_wrt_Csi(C_ij, C_si, C_sj, step_size, threshold)
% check_jac_wrt_Csj(C_ij, C_si, C_sj, step_size, threshold)
