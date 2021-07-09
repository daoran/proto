#!/usr/bin/octave
graphics_toolkit("fltk");

################################# UTILS ######################################

function A = skew(x)
  assert(size(x) == [3, 1]);
  A = [0, -x(3), x(2);
       x(3), 0, -x(1);
       -x(2), x(1), 0];
endfunction

function w = skew_inv(A)
  assert(size(A) == [3, 3]);
  w = [A(3, 2); A(1, 3); A(2, 1)];
endfunction

function R = euler321(rpy)
  phi = rpy(1);
  theta = rpy(2);
  psi = rpy(3);

  R11 = cos(psi) * cos(theta);
  R21 = sin(psi) * cos(theta);
  R31 = -sin(theta);

  R12 = cos(psi) * sin(theta) * sin(phi) - sin(psi) * cos(phi);
  R22 = sin(psi) * sin(theta) * sin(phi) + cos(psi) * cos(phi);
  R32 = cos(theta) * sin(phi);

  R13 = cos(psi) * sin(theta) * cos(phi) + sin(psi) * sin(phi);
  R23 = sin(psi) * sin(theta) * cos(phi) - cos(psi) * sin(phi);
  R33 = cos(theta) * cos(phi);

  R = [R11, R12, R13; R21, R22, R23; R31, R32, R33;];
endfunction

function euler = quat2euler(q)
  qw = q(1);
  qx = q(2);
  qy = q(3);
  qz = q(4);

  qw2 = qw**2;
  qx2 = qx**2;
  qy2 = qy**2;
  qz2 = qz**2;

  t1 = atan2(2 * (qx * qw + qz * qy), (qw2 - qx2 - qy2 + qz2));
  t2 = asin(2 * (qy * qw - qx * qz));
  t3 = atan2(2 * (qx * qy + qz * qw), (qw2 + qx2 - qy2 - qz2));
  euler = [t1; t2; t3];
endfunction

function n = quat_norm(q)
  n = sqrt(q(1)**2 + q(2)**2 + q(3)**2 + q(4)**2);
endfunction

function q_out = quat_normalize(q)
  n = quat_norm(q);
  q_out = [q(1) / n; q(2) / n; q(3) / n; q(4) / n];
endfunction

function r = quat_lmul(p, q)
  assert(size(p) == [4, 1]);
  assert(size(q) == [4, 1]);

  pw = p(1);
  px = p(2);
  py = p(3);
  pz = p(4);

  lprod = [
    pw, -px, -py, -pz;
    px, pw, -pz, py;
    py, pz, pw, -px;
    pz, -py, px, pw;
  ];

  r = lprod * q;
endfunction

function r = quat_mul(p, q)
  r = quat_lmul(p, q);
endfunction

function q_inv = quat_inv(q)
  % assert(norm(q) == 1.0);
  q_inv = quat_conj(q);
endfunction

function q_conj = quat_conj(q)
  qw = q(1);
  qx = q(2);
  qy = q(3);
  qz = q(4);
  q_conj = [qw; -qx; -qy; -qz];
endfunction

function q_kp1 = quat_integrate(q_k, w, dt)
  % "Quaternion kinematics for the error-state Kalman filter" (2017)
  % By Joan Sola
  % [Section 4.6.1 Zeroth-order integration, p.47]
  w_norm = norm(w);
  q_scalar = 0.0;
  q_vec = [0.0; 0.0; 0.0];

  if (w_norm > 1e-5)
    q_scalar = cos(w_norm * dt * 0.5);
    q_vec = w / w_norm * sin(w_norm * dt * 0.5);
  else
    q_scalar = 1.0;
    q_vec = sin(w_norm * dt * 0.5);
  endif

  q_kp1 = quat_mul(q_k, [q_scalar; q_vec]);
endfunction

function dq = quat_delta(dalpha)
  half_norm = 0.5 * norm(dalpha);
  scalar = cos(half_norm);
  vector = sinc(half_norm) * 0.5 * dalpha;
  dq = [scalar; vector];
endfunction

function q = rot2quat(R)
  m00 = R(1, 1);
  m01 = R(1, 2);
  m02 = R(1, 3);

  m10 = R(2, 1);
  m11 = R(2, 2);
  m12 = R(2, 3);

  m20 = R(3, 1);
  m21 = R(3, 2);
  m22 = R(3, 3);

  tr = m00 + m11 + m22;

  if (tr > 0)
    S = sqrt(tr+1.0) * 2; % S=4*qw
    qw = 0.25 * S;
    qx = (m21 - m12) / S;
    qy = (m02 - m20) / S;
    qz = (m10 - m01) / S;
  elseif ((m00 > m11) && (m00 > m22))
    S = sqrt(1.0 + m00 - m11 - m22) * 2; % S=4*qx
    qw = (m21 - m12) / S;
    qx = 0.25 * S;
    qy = (m01 + m10) / S;
    qz = (m02 + m20) / S;
  elseif (m11 > m22)
    S = sqrt(1.0 + m11 - m00 - m22) * 2; % S=4*qy
    qw = (m02 - m20) / S;
    qx = (m01 + m10) / S;
    qy = 0.25 * S;
    qz = (m12 + m21) / S;
  else
    S = sqrt(1.0 + m22 - m00 - m11) * 2; % S=4*qz
    qw = (m10 - m01) / S;
    qx = (m02 + m20) / S;
    qy = (m12 + m21) / S;
    qz = 0.25 * S;
  endif

  q = quat_normalize([qw; qx; qy; qz]);
endfunction

function R = quat2rot(q)
  qw = q(1);
  qx = q(2);
  qy = q(3);
  qz = q(4);

  qx2 = qx**2;
  qy2 = qy**2;
  qz2 = qz**2;
  qw2 = qw**2;

  # Homogeneous form
  R11 = qw2 + qx2 - qy2 - qz2;
  R12 = 2 * (qx * qy - qw * qz);
  R13 = 2 * (qx * qz + qw * qy);

  R21 = 2 * (qx * qy + qw * qz);
  R22 = qw2 - qx2 + qy2 - qz2;
  R23 = 2 * (qy * qz - qw * qx);

  R31 = 2 * (qx * qz - qw * qy);
  R32 = 2 * (qy * qz + qw * qx);
  R33 = qw2 - qx2 - qy2 + qz2;

  R = [R11, R12, R13; R21, R22, R23; R31, R32, R33;];
endfunction

function euler = rot2euler(R)
  q = rot2quat(R);
  euler = quat2euler(q);
endfunction

function T = tf(varargin)
  rot = eye(3);
  trans = zeros(3, 1);

  % Parse arguments
  assert(length(varargin) == 1 || length(varargin) == 2);
  if length(varargin) == 1
    pose = varargin{1};
    assert(all(size(pose) == [7, 1]));
    rot = quat2rot(pose(1:4));
    trans = pose(5:7);

  elseif length(varargin) == 2
    rot = varargin{1};
    trans = varargin{2};
    assert(size(rot) == [3, 3] || size(rot) == [4, 1]);
    assert(size(trans) == [3, 1]);
    if size(rot) == [4, 1]
      rot = quat2rot(rot);
    endif

  endif

  T = eye(4, 4);
  T(1:3, 1:3) = rot;
  T(1:3, 4) = trans;
endfunction

function R = rvec2rot(rvec)
  % If small rotation
  theta = sqrt(rvec(:)'*rvec(:));  % = norm(rvec), but faster
  if theta < eps
    R = [1, -rvec(3), rvec(2);
          rvec(3), 1, -rvec(1);
          -rvec(2), rvec(1), 1];
    return
  end

  % Convert rvec to rotation matrix
  rvec = rvec / theta;
  x = rvec(1);
  y = rvec(2);
  z = rvec(3);

  c = cos(theta);
  s = sin(theta);
  C = 1 - c;

  xs = x * s;
  ys = y * s;
  zs = z * s;

  xC = x * C;
  yC = y * C;
  zC = z * C;

  xyC = x * yC;
  yzC = y * zC;
  zxC = z * xC;

  R = [x * xC + c, xyC - zs, zxC + ys;
       xyC + zs, y * yC + c, yzC - xs;
       zxC - ys, yzC + xs, z * zC + c];
  return
endfunction

function r = tf_trans(T)
  r = T(1:3, 4);
endfunction

function q = tf_quat(tf)
  q = rot2quat(tf(1:3, 1:3));
endfunction

function C = tf_rot(tf)
  C = tf(1:3, 1:3);
endfunction

function [T_diff] = perturb_trans(T, step_size, i)
  assert(size(T) == [4, 4] || size(T) == [3, 1]);

  if (size(T) == [4, 4])
    dr = eye(3) * step_size;
    C = tf_rot(T);
    r = tf_trans(T);
    r_diff = r + dr(1:3, i);
    T_diff = tf(C, r_diff);

  elseif (size(T) == [3, 1])
    r = T;
    dr = eye(3) * step_size;
    r_diff = r + dr(1:3, i);
    T_diff = r_diff;

  endif
endfunction

function [retval] = perturb_rot(T, step_size, i)
  assert(size(T) == [4, 4] || size(T) == [3, 3]);

  if (size(T) == [4, 4])
    C = tf_rot(T);
    r = tf_trans(T);

    rvec = eye(3) * step_size;
    C_diff = rvec2rot(rvec(1:3, i));
    C_diff = C_diff * C;
    retval = tf(C_diff, r);

  elseif (size(T) == [3, 3])
    rvec = eye(3) * step_size;
    C = T;
    C_diff = rvec2rot(rvec(1:3, i));
    retval = C_diff * C;

  endif
endfunction

function [retval] = perturb_pose(T, step_size, i)
  assert(size(T) == [4, 4] && i >= 0 && i <= 6);
  if i <= 3
    retval = perturb_trans(T, step_size, i);
  else
    retval = perturb_rot(T, step_size, i - 3);
  endif
endfunction

function C = Exp(phi)
  assert(size(phi) == [3, 1]);
  if (phi < 1e-3)
    C = eye(3) + skew(phi);
  else
    C = eye(3);
    C += (sin(norm(phi)) / norm(phi)) * skew(phi);
    C += ((1 - cos(norm(phi))) / norm(phi)^2) * skew(phi)^2;
  endif
endfunction

function rvec = Log(C)
  assert(size(C) == [3, 3]);
  phi = acos(trace(C) - 1 / 2);
  rvec = (phi * skew_inv(C - C')) / (2 * sin(phi));
endfunction

function J = Jr(theta)
  % Equation (8) in:
  % Forster, Christian, et al. "IMU preintegration on manifold for efficient
  % visual-inertial maximum-a-posteriori estimation." Georgia Institute of
  % Technology, 2015.
  theta_norm = norm(theta);
  theta_norm_sq = theta_norm * theta_norm;
  theta_norm_cube = theta_norm_sq * theta_norm;
  theta_skew = skew(theta);
  theta_skew_sq = theta_skew * theta_skew;

  J = eye(3);
  J -= ((1 - cos(theta_norm)) / theta_norm_sq) * theta_skew;
  J += (theta_norm - sin(theta_norm)) / (theta_norm_cube) * theta_skew_sq;
endfunction

function J = Jr_inv(theta)
  theta_norm = norm(theta);
  theta_norm_sq = theta_norm * theta_norm;
  theta_skew = skew(theta);
  theta_skew_sq = theta_skew * theta_skew;

  A = 1.0 / theta_norm_sq;
  B = (1 + cos(theta_norm)) / (2 * theta_norm * sin(theta_norm));

  J = eye(3);
  J += 0.5 * theta_skew;
  J += (A - B) * theta_skew_sq;
endfunction

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

% Imu data
dt = 0.01;
dt_sq = dt * dt;
acc_m = [0.01; 0.02; 0.03];
gyr_m = [0.01; 0.02; 0.03];
ba = [1e-5; 1e-5; 1e-5];
bg = [1e-5; 1e-5; 1e-5];
a = acc_m - ba;
w = gyr_m - bg;

% Propagate IMU state using Euler method
r_i = zeros(3, 1);
v_i = zeros(3, 1);
C_i = eye(3, 3);
r_j = r_i + (v_i * dt) + (0.5 * (C_i * (a - bg)) * dt_sq);
v_j = v_i + C_i * (a - bg) * dt;
C_j = C_i * Exp(w * dt);

% Continuous time transition matrix F
F = zeros(15, 15);
F(1:3, 4:6) = eye(3);
F(4:6, 7:9) = -C_i * skew(a);
F(4:6, 10:12) = -C_i;
F(7:9, 7:9) = -skew(w);
F(7:9, 13:15) = -eye(3);

err_pos = (C_i' * ((r_j - r_i) - (v_i * dt))) - 
