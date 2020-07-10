#!/bin/env octave -qf
graphics_toolkit("fltk");

cam_pose_csv = "/tmp/sim_data/cam0_pose.csv";
imu_pose_csv = "/tmp/sim_data/imu_pose.csv";
imu_vel_csv = "/tmp/sim_data/imu_vel.csv";

cam_pose_data = csvread(cam_pose_csv, 0, 0);
imu_pose_data = csvread(imu_pose_csv, 0, 0);
imu_vel_data = csvread(imu_vel_csv, 0, 0);

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

% Parse imu pose
imu_ts = [];
imu_pos = [];
imu_rot = [];
for i = 1:rows(imu_pose_data)
  data = imu_pose_data(i, :);
  imu_ts = [imu_ts; data(1)];
  imu_pos = [imu_pos; data(6:8)];
  imu_rot = [imu_rot; quat2euler(data(2:6))'];
endfor

% Parse imu velocity
imu_vel = [];
for i = 1:rows(imu_vel_data)
  data = imu_vel_data(i, :);
  imu_vel = [imu_vel; data(2:4)];
endfor

% Parse cam pose
cam_ts = [];
cam_pos = [];
cam_rot = [];
for i = 1:rows(cam_pose_data)
  data = cam_pose_data(i, :);
  cam_ts = [cam_ts; data(1)];
  cam_pos = [cam_pos; data(6:8)];
  cam_rot = [cam_rot; quat2euler(data(2:6))'];
endfor


% Plot position and attitude
figure(1);
subplot(211);
hold on;
ts = imu_ts * 1e-9;
plot(ts, imu_pos(:, 1), "r.", "markersize", 20);
plot(ts, imu_pos(:, 2), "b.", "markersize", 20);
plot(ts, imu_pos(:, 3), "g.", "markersize", 20);
legend("x", "y", "z");
xlabel("Time [s]");
ylabel("Position [m]");
xlim([0, max(ts)]);
title("Position");

subplot(212);
hold on;
ts = imu_ts * 1e-9;
plot(ts, imu_rot(:, 1), "r.", "markersize", 20);
plot(ts, imu_rot(:, 2), "b.", "markersize", 20);
plot(ts, imu_rot(:, 3), "g.", "markersize", 20);
legend("x", "y", "z");
xlabel("Time [s]");
ylabel("Attitude [deg]");
xlim([0, max(ts)]);
title("Attitude");

% Plot velocity
figure(2);
hold on;
ts = imu_ts * 1e-9;
plot(ts, imu_vel(:, 1), "r.", "markersize", 20);
plot(ts, imu_vel(:, 2), "b.", "markersize", 20);
plot(ts, imu_vel(:, 3), "g.", "markersize", 20);
legend("x", "y", "z");
xlabel("Time [s]");
ylabel("Velocity [ms^-1]");
xlim([0, max(ts)]);
title("Velocity");

% Plot 3D
figure(3);
hold on;
plot3(imu_pos(1:100:end, 1), imu_pos(1:100:end, 2), imu_pos(1:100:end, 3), 'r.');

for k = 1:int32(rows(imu_pos)*0.1):rows(imu_pos)
  pos = transpose(imu_pos(k, :));
  rot = euler321(imu_rot(k, :));
  T_WS = tf(rot, pos);
  draw_frame(T_WS);
endfor

% for k = 1:int32(rows(cam_pos)*0.01):rows(cam_pos)
%   pos = transpose(cam_pos(k, :));
%   rot = euler321(cam_rot(k, :));
%   T_WS = tf(rot, pos);
%   draw_frame(T_WS);
% endfor

% rpy_WC = [deg2rad(-90.0); deg2rad(0.0); deg2rad(-90.0)];
% C_WC = euler321(rpy_WC);
% r_WC = [0; 0; 1.0];
% T_WC = tf(C_WC, r_WC);
% draw_frame(T_WC);

title("3D plot");
xlabel("x");
ylabel("y");
zlabel("z");
axis "equal";
view(3);
ginput();
