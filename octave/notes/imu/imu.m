#!/usr/bin/octave
graphics_toolkit("fltk");

################################# UTILS ######################################

function y = skew(x)
  y = [0, -x(3), x(2);
       x(3), 0, -x(1);
       -x(2), x(1), 0];
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

function r = tf_trans(T)
  r = T(1:3, 4);
endfunction

function q = tf_quat(tf)
  q = rot2quat(tf(1:3, 1:3));
endfunction

function C = tf_rot(tf)
  C = tf(1:3, 1:3);
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

################################### IMU ######################################

function x_imu = imu_state_init()
  x_imu.p_WS = zeros(3, 1);
  x_imu.v_WS = zeros(3, 1);
  x_imu.C_WS = eye(3);
  x_imu.ba = zeros(3, 1);
  x_imu.bg = zeros(3, 1);
  x_imu.g = [0; 0; 9.81];
endfunction

function x_imu = imu_euler_update(x_imu, acc, gyr, dt)
  g = x_imu.g;
  ba = x_imu.ba;
  bg = x_imu.bg;

  C_WS_i = x_imu.C_WS;
  v_WS_i = x_imu.v_WS;

  w = gyr - bg;
  a = acc - ba;
  dt_sq = dt * dt;

  x_imu.C_WS *= so3_exp(w * dt);
  x_imu.v_WS += ((C_WS_i * a) - g) * dt;
  x_imu.p_WS += (v_WS_i * dt) + (0.5 * (C_WS_i * a - g) * dt_sq);
endfunction

function x_imu = imu_rk4_update(x_imu, acc, gyr, dt)
  g = x_imu.g;
  ba = x_imu.ba;
  bg = x_imu.bg;
  na = zeros(3, 1);
  ng = zeros(3, 1);

  C_WS_k = x_imu.C_WS;
  q_WS_k = rot2quat(C_WS_k);
  v_WS_k = x_imu.v_WS;
  r_k = x_imu.p_WS;

  w = gyr - bg;
  a = acc - ba;

  % Runge-Kutta 4th Order
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
  k2_v_dot = C_WS_kphdt * acc - g;
  k2_p_dot = v_WS_k + k1_v_dot * dt / 2;
  % -- k3 = f(tn + dt / 2, yn + k2 * dt / 2)
  k3_v_dot = C_WS_kphdt * acc - g;
  k3_p_dot = v_WS_k + k2_v_dot * dt / 2;
  % -- k4 = f(tn + dt, tn + k3 * dt)
  k4_v_dot = C_WS_kpdt * acc - g;
  k4_p_dot = v_WS_k + k3_v_dot * dt;

  x_imu.C_WS = C_WS_kpdt;
  x_imu.v_WS += dt / 6 * (k1_v_dot + 2 * k2_v_dot + 2 * k3_v_dot + k4_v_dot);
  x_imu.p_WS += dt / 6 * (k1_p_dot + 2 * k2_p_dot + 2 * k3_p_dot + k4_p_dot);
endfunction

function est = imu_batch_integrate(sim_data, method)
  printf("Performing batch IMU integration using [%s] method\n", method);

  % Initialize IMU state
  x_imu = imu_state_init();
  x_imu.p_WS = sim_data.pos(:, 1);
  x_imu.v_WS = sim_data.vel(:, 1);
  x_imu.C_WS = tf_rot(sim_data.poses{1});

  % Batch integrate IMU measurements
  est_pos = [x_imu.p_WS];
  est_vel = [x_imu.v_WS];
  est_att = [rot2euler(x_imu.C_WS)];
  t_prev = sim_data.time(1);
  for k = 2:length(sim_data.time)
    % Calculate dt
    t = sim_data.time(k);
    dt = t - t_prev;

    % Propagate IMU state
    acc = sim_data.imu_acc(:, k);
    gyr = sim_data.imu_gyr(:, k);
    if strcmp(method, "euler") == 1
      x_imu = imu_euler_update(x_imu, acc, gyr, dt);
    elseif strcmp(method, "rk4") == 1
      x_imu = imu_rk4_update(x_imu, acc, gyr, dt);
    else
      printf("Invalid method [%s]!\n", method);
      exit(-1);
    endif

    % Keep track of t_prev
    est_pos = [est_pos, x_imu.p_WS];
    est_vel = [est_vel, x_imu.v_WS];
    est_att = [est_att, rot2euler(x_imu.C_WS)];
    t_prev = t;
  endfor

  est = {};
  est.time = sim_data.time;
  est.pos = est_pos;
  est.vel = est_vel;
  est.att = est_att;
endfunction

function imu_preintegrate(imu_ts, imu_acc, imu_gyr, g,
                          pose_i, sb_i, pose_j, sb_j)
  dC = eye(3, 3);
  dq = rot2quat(dC);
  dv = zeros(3, 1);
  dr = zeros(3, 1);
  ba = sb_i(4:6);
  bg = sb_i(7:9);

  t_prev = imu_ts(1);
  dt = 0.0;
  Dt = 0.0;

  J = zeros(15, 15);
  P = zeros(15, 15);
  for k = 2:length(imu_ts)
    % Calculate dt
    t = imu_ts(k);
    dt = t - t_prev;

    % Propagate IMU state using Runge-Kutta 4th Order
    a = imu_acc(:, k) - ba;
    w = imu_gyr(:, k) - bg;
    % -- Integrate orientation at time k + dt (kpdt: k plus dt)
    dq_kpdt = quat_integrate(dq, w, dt);
    dC_kpdt = quat2rot(dq_kpdt);
    % -- Integrate orientation at time k + dt / 2 (kphdt: k plus half dt)
    dq_kphdt = quat_integrate(dq, w, dt / 2);
    dC_kphdt = quat2rot(dq_kphdt);
    % -- k1 = f(tn, yn)
    k1_v_dot = dC * a - g;
    k1_p_dot = dv;
    % -- k2 = f(tn + dt / 2, yn + k1 * dt / 2)
    k2_v_dot = dC_kphdt * a - g;
    k2_p_dot = dv + k1_v_dot * dt / 2;
    % -- k3 = f(tn + dt / 2, yn + k2 * dt / 2)
    k3_v_dot = dC_kphdt * a - g;
    k3_p_dot = dv + k2_v_dot * dt / 2;
    % -- k4 = f(tn + dt, tn + k3 * dt)
    k4_v_dot = dC_kpdt * a - g;
    k4_p_dot = dv + k3_v_dot * dt;
    % -- Put it all together
    dC = dC_kpdt;
    dv += dt / 6 * (k1_v_dot + 2 * k2_v_dot + 2 * k3_v_dot + k4_v_dot);
    dr += dt / 6 * (k1_p_dot + 2 * k2_p_dot + 2 * k3_p_dot + k4_p_dot);

    % Continuous time transition matrix F
    F = zeros(15, 15);
    F(1:3, 1:3) = -skew(w);
    F(1:3, 4:6) = -eye(3);
    F(7:9, 1:3) = -dC * skew(a);
    F(7:9, 10:12) = -dC;
    F(13:15, 7:9) = eye(3);

    % Continuous time input matrix G
    G = zeros(18, 12);
    G(4:6, 1:3) = -dC;
    G(7:9, 4:6) = -eye(3);
    G(10:12, 7:9) = eye(3);
    G(13:15, 10:12) = eye(3);

    % Discretize transition matrix F by using Taylor Series to the 3rd order
    % "Quaternion kinematics for the error-state Kalman filter" (2017)
    % By Joan Sola
    % [Section B, p.73 "Closed-form Integration Methods"]
    F_dt = F * dt;
    Phi = eye(15);
    Phi += F_dt;
    Phi += (1.0 / 2.0) * F_dt^2;
    Phi += (1.0 / 6.0) * F_dt^3;

    % % Propagate the state covariance matrix
    % Q_k = Phi * G * Q * G' * Phi' * dt;
    % P = Phi * P * Phi' + Q_k;

    % % Update
    % J = Phi * J;
    t_prev = t;
    Dt += dt;
  endfor

  r_i = tf_trans(pose_i);
  r_j = tf_trans(pose_j);
  C_i = tf_rot(pose_i);
  v_i = sb_i(1:3);
  ba_i = sb_i(4:6);
  bg_i = sb_i(4:6);

  C_j = tf_rot(pose_j);
  v_j = sb_j(1:3);
  ba_j = sb_j(4:6);
  bg_j = sb_j(4:6);

  % dC
  % dv
  % dr

  Dt_sq = Dt * Dt;
  err_pos = C_i' * ((r_j - r_i) - (v_i * Dt) + (0.5 * g * Dt_sq)) - dr
  err_vel = C_i' * ((v_j - v_i) + (g * Dt)) - dv
  err_rot = C_i' * C_j;
  err_ba = ba_j - ba_i
  err_bg = bg_j - bg_i
  % err = [err_pos; err_vel; err_rot; err_ba; err_bg];

endfunction

function plot_imu(sim_data)
  figure();
  clf;

  subplot(211);
  hold on;
  plot(sim_data.time, sim_data.imu_acc(1, :), "r-", "linewidth", 2);
  plot(sim_data.time, sim_data.imu_acc(2, :), "g-", "linewidth", 2);
  plot(sim_data.time, sim_data.imu_acc(3, :), "b-", "linewidth", 2);
  title("Accelerometer");
  xlabel("Time [s]");
  ylabel("Acceleration [m s^-2]");
  xlim([0, sim_data.time(end)]);

  subplot(212);
  hold on;
  plot(sim_data.time, sim_data.imu_gyr(1, :), "r-", "linewidth", 2);
  plot(sim_data.time, sim_data.imu_gyr(2, :), "g-", "linewidth", 2);
  plot(sim_data.time, sim_data.imu_gyr(3, :), "b-", "linewidth", 2);
  title("Gyroscope");
  xlabel("Time [s]");
  ylabel("Angular Velocity [rad s^-1]");
  xlim([0, sim_data.time(end)]);
endfunction

function plot_poses(gnd, est=0)
  figure();
  clf;

  subplot(4, 1, [3 1]);
  hold on;
  plot(gnd.pos(1, :), gnd.pos(2, :), 'r-', "linewidth", 2);
  plot(est.pos(1, :), est.pos(2, :), 'r--', "linewidth", 2);
  axis("equal");
  title("Position X-Y");
  xlabel("x [m]");
  ylabel("y [m]");
  legend("Ground Truth", "Estimated");

  subplot(4, 1, 4);
  hold on;
  plot(gnd.time, gnd.att(3, :), 'r-', "linewidth", 2);
  plot(est.time, est.att(3, :), 'r--', "linewidth", 2);
  xlim([0, max(gnd.time(end), est.time(end))]);
  title("Altitude");
  xlabel("time [s]");
  ylabel("z [m]");
  legend("Ground Truth", "Estimated");
endfunction

function sim_data = sim_imu(circle_r, velocity)
  imu_rate = 200.0;
  circle_dist = 2.0 * pi * circle_r;
  time_taken = circle_dist / velocity;
  g = [0.0; 0.0; 9.81];
  printf("Simulating ideal IMU measurements ...\n");
  printf("imu_rate: %f\n", imu_rate);
  printf("circle_r: %f\n", circle_r);
  printf("circle_dist: %f\n", circle_dist);
  printf("time_taken: %f\n", time_taken);

  dt = 1.0 / imu_rate;
  w = -2.0 * pi * (1.0 / time_taken);
  t = 0;

  theta = pi;
  yaw = 0;

  sensor_poses = {};
  sensor_pos = [];
  sensor_quat = [];
  sensor_att = [];
  sensor_vel = [];

  imu_ts = [];
  imu_acc = [];
  imu_gyr = [];

  idx = 1;
  while (t <= time_taken)
    % Sensor pose
    rx = circle_r * cos(theta);
    ry = circle_r * sin(theta);
    rz = 0.0;
    r_WS = [rx; ry; rz];
    rpy_WS = [0.0; 0.0; yaw];
    C_WS = euler321(rpy_WS);
    T_WS = tf(C_WS, r_WS);

    % Sensor velocity
    vx = -circle_r * w * sin(theta);
    vy = circle_r * w * cos(theta);
    vz = 0.0;
    v_WS = [vx; vy; vz];

    % Sensor acceleration
    ax = -circle_r * w * w * cos(theta);
    ay = -circle_r * w * w * sin(theta);
    az = 0.0;
    a_WS = [ax; ay; az];

    % Sensor angular velocity
    wx = 0.0;
    wy = 0.0;
    wz = w;
    w_WS = [wx; wy; wz];

    % IMU measurements
    acc = C_WS' * (a_WS + g);
    gyr = C_WS' * w_WS;

    % Update
    sensor_poses{idx} = T_WS;
    sensor_pos = [sensor_pos, tf_trans(T_WS)];
    sensor_quat = [sensor_quat, tf_quat(T_WS)];
    sensor_att = [sensor_att, quat2euler(tf_quat(T_WS))];
    sensor_vel = [sensor_vel, v_WS];

    imu_ts = [imu_ts; t];
    imu_acc = [imu_acc, acc];
    imu_gyr = [imu_gyr, gyr];

    idx += 1;
    theta += w * dt;
    yaw += w * dt;
    t += dt;

    % if (t >= (time_taken * 0.3))
    %   break;
    % end
  endwhile

  sim_data = {};
  sim_data.poses = sensor_poses;
  sim_data.pos = sensor_pos;
  sim_data.quat = sensor_quat;
  sim_data.att = sensor_att;
  sim_data.vel = sensor_vel;
  sim_data.time = imu_ts;
  sim_data.imu_acc = imu_acc;
  sim_data.imu_gyr = imu_gyr;
endfunction

function traj_error(gnd, est)
  assert(length(gnd.time) == length(est.time));

  err_x = [];
  err_y = [];
  err_z = [];

  for k = 1:length(gnd.time)
    assert(gnd.time(k) == est.time(k));
    gnd_pos = gnd.pos(:, k);
    est_pos = est.pos(:, k);
    err_x = [err_x, gnd_pos(1) - est_pos(1)];
    err_y = [err_y, gnd_pos(2) - est_pos(2)];
    err_z = [err_z, gnd_pos(3) - est_pos(3)];
  endfor

  dist = norm(gnd.pos(:, end) - est.pos(:, end));
  printf("Euclidean distance between start and end: %f\n", dist);

  printf("Error (x, y, z):\n");
  printf("mean: [%f, %f, %f]\n", mean(err_x), mean(err_y), mean(err_z));
  printf("median: [%f, %f, %f]\n", median(err_x), median(err_y), median(err_z));
  printf("var: [%f, %f, %f]\n", var(err_x), var(err_y), var(err_z));
endfunction

################################### MAIN ######################################

sim_data = sim_imu(0.5, 1.0);
% est = imu_batch_integrate(sim_data, "euler");
est = imu_batch_integrate(sim_data, "rk4");
traj_error(sim_data, est);

% N = 100;
% imu_ts = sim_data.time(1:N);
% imu_acc = sim_data.imu_acc(:, 1:N);
% imu_gyr = sim_data.imu_gyr(:, 1:N);
% g = [0.0; 0.0; 9.81];
% pose_i = sim_data.poses{1}
% sb_i = [sim_data.vel(:, 1); zeros(6, 1)];
% pose_j = sim_data.poses{N}
% sb_j = [sim_data.vel(:, N); zeros(6, 1)];
%
% imu_preintegrate(imu_ts, imu_acc, imu_gyr, g,
%                  pose_i, sb_i, pose_j, sb_j);

% Print errors
plot_poses(sim_data, est);
plot_imu(sim_data);
ginput();
