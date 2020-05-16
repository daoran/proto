#!/bin/env octave -qf
graphics_toolkit("fltk");

cam_pose_csv = "/tmp/sim_data/cam0_pose.csv";
cam_pose_est_csv = "/tmp/sim_data/cam0_pose_est.csv";

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

function T = tf(rot, trans)
  assert(size(rot) == [3, 3] || size(rot) == [4, 1]);
  assert(size(trans) == [3, 1]);

  C = rot;
  if size(rot) == [4, 1]
    C = quat2rot(rot);
  endif

  T = eye(4, 4);
  T(1:3, 1:3) = C;
  T(1:3, 4) = trans;
endfunction

function r = tf_trans(T)
  r = T(1:3, 4);
endfunction

function hp = homogeneous(p)
  hp = [p; ones(1, columns(p))];
endfunction

function draw_frame(T_WB, scale=1.1)
  r_WB = tf_trans(T_WB);
  origin = r_WB;

  x_axis = T_WB * homogeneous(scale * [1; 0; 0]);
  y_axis = T_WB * homogeneous(scale * [0; 1; 0]);
  z_axis = T_WB * homogeneous(scale * [0; 0; 1]);

  % Draw x-axis
  plot3([origin(1), x_axis(1)], ...
        [origin(2), x_axis(2)], ...
        [origin(3), x_axis(3)], 'r',
        "linewidth", 5)

  % Draw y-axis
  plot3([origin(1), y_axis(1)], ...
        [origin(2), y_axis(2)], ...
        [origin(3), y_axis(3)], 'g',
        "linewidth", 5)

  % Draw z-axis
  plot3([origin(1), z_axis(1)], ...
        [origin(2), z_axis(2)], ...
        [origin(3), z_axis(3)], 'b',
        "linewidth", 5)
endfunction

function [ts, pos, rot] = parse_pose(csv_path)
  csv_data = csvread(csv_path, 0, 0);

  ts = [];
  pos = [];
  rot = [];
  for i = 1:rows(csv_data)
    data = csv_data(i, :);
    ts = [ts; data(1)];
    pos = [pos; data(6:8)];
    rot = [rot; quat2euler(data(2:6))'];
  endfor
endfunction

function [ts, vel] = parse_vel(csv_path)
  csv_data = csvread(csv_path, 0, 0);

  ts = [];
  vel = [];
  for i = 1:rows(csv_data)
    data = csv_data(i, :);
    ts = [ts; data(1)];
    vel = [vel; data(2:4)];
  endfor
endfunction

% Parse data
[cam_ts, cam_pos, cam_rot] = parse_pose(cam_pose_csv);
[cam_est_ts, cam_est_pos, cam_est_rot] = parse_pose(cam_pose_est_csv);


% Plot position
figure(1);
nb_meas = length(cam_est_ts);
ts = cam_ts(1:nb_meas) * 1e-9;

subplot(311);
hold on;
plot(ts, cam_pos(1:nb_meas, 1), "r-", "linewidth", 1.5);
plot(ts, cam_est_pos(:, 1), "rx", "linewidth", 1.0);
xlabel("Time [s]");
ylabel("Position [m]");
xlim([0, max(ts)]);

subplot(312);
hold on;
plot(ts, cam_pos(1:nb_meas, 2), "g-", "linewidth", 1.5);
plot(ts, cam_est_pos(:, 2), "gx", "linewidth", 1.0);
xlabel("Time [s]");
ylabel("Position [m]");
xlim([0, max(ts)]);

subplot(313);
hold on;
plot(ts, cam_pos(1:nb_meas, 3), "b-", "linewidth", 1.5);
plot(ts, cam_est_pos(:, 3), "bx", "linewidth", 1.0);
xlabel("Time [s]");
ylabel("Position [m]");
xlim([0, max(ts)]);


% Plot attitude
figure(2);
nb_meas = length(cam_est_ts);
ts = cam_ts(1:nb_meas) * 1e-9;

subplot(311);
hold on;
plot(ts, cam_rot(1:nb_meas, 1), "r-", "linewidth", 1.5);
plot(ts, cam_est_rot(:, 1), "rx", "linewidth", 1.5);
xlabel("Time [s]");
ylabel("Attitude [deg]");
xlim([0, max(ts)]);

subplot(312);
hold on;
plot(ts, cam_rot(1:nb_meas, 2), "g-", "linewidth", 1.5);
plot(ts, cam_est_rot(:, 2), "gx", "linewidth", 1.5);
xlabel("Time [s]");
ylabel("Attitude [deg]");
xlim([0, max(ts)]);

subplot(313);
hold on;
plot(ts, cam_rot(1:nb_meas, 3), "b-", "linewidth", 1.5);
plot(ts, cam_est_rot(:, 3), "bx", "linewidth", 1.5);
xlabel("Time [s]");
ylabel("Attitude [deg]");
xlim([0, max(ts)]);
