#!/usr/bin/octave -qf
graphics_toolkit("fltk");

features_path = "/tmp/features.csv";
cam_poses_path = "/tmp/cam_poses.csv";
imu_path = "/tmp/imu.csv";
imu_poses_path = "/tmp/imu_poses.csv";

features = csvread(features_path);
cam_poses = csvread(cam_poses_path);
imu_data = csvread(imu_path);
imu_poses = csvread(imu_poses_path);

function hp = homogeneous(p)
  hp = [p; ones(1, columns(p))];
endfunction

function draw_camera(T_WC, scale=0.1, style="b-")
  fov = deg2rad(60.0);

  # Form the camera fov frame
  fov_hwidth = scale;
  fov_corners = zeros(3, 4);
  fov_corners(1:3, 1) = [-fov_hwidth; fov_hwidth; 0.0];  # Bottom left
  fov_corners(1:3, 2) = [-fov_hwidth; -fov_hwidth; 0.0]; # Top left
  fov_corners(1:3, 3) = [fov_hwidth; -fov_hwidth; 0.0];  # Top right
  fov_corners(1:3, 4) = [fov_hwidth; fov_hwidth; 0.0];   # Bottom right

  # Calculate the distance from camera origin to fov frame given fov
  dist = fov_hwidth / tan(fov / 2.0);
  fov_corners(3, :) = dist;

  # Transform fov_corners to world frame
  fov_corners = T_WC * [fov_corners; ones(1, 4)];
  fov_corners = fov_corners(1:3, :);

  # Transform camera_origin to world frame
  cam_origin = [0; 0; 0];
  cam_origin = T_WC * [cam_origin; 1.0];
  cam_origin = cam_origin(1:3, :);

  # Draw fov frame
  frame_x = [fov_corners(1, :), fov_corners(1, 1)];
  frame_y = [fov_corners(2, :), fov_corners(2, 1)];
  frame_z = [fov_corners(3, :), fov_corners(3, 1)];
  plot3(frame_x, frame_y, frame_z, style);

  # Draw from camera origin to fov frame
  for i = 1:4
    x = [cam_origin(1), fov_corners(1, i)];
    y = [cam_origin(2), fov_corners(2, i)];
    z = [cam_origin(3), fov_corners(3, i)];
    plot3(x, y, z, style);
  endfor
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

function plot_3d(fig_id, plot_title, features, cam_poses)
  figure(fig_id);
  hold on;

  scatter3(features(:, 1), features(:, 2), features(:, 3), '.');

  for i = 1:10:rows(cam_poses)
    r_WC = cam_poses(i, 2:4)';
    q_WC = cam_poses(i, 5:8)';
    T_WC = tf(q_WC, r_WC);
    draw_camera(T_WC, 0.5);
    draw_frame(T_WC);
  endfor

  view(3);
  xlabel("x [m]");
  ylabel("y [m]");
  zlabel("z [m]");
  title(plot_title);
  axis 'equal';
endfunction

function plot_poses(fig_id, plot_title, data)
  figure(fig_id);

  subplot(211);
  hold on;
  plot(data(:, 1) * 1e-9, data(:, 2), 'r-', 'linewidth', 2.0);
  plot(data(:, 1) * 1e-9, data(:, 3), 'g-', 'linewidth', 2.0);
  plot(data(:, 1) * 1e-9, data(:, 4), 'b-', 'linewidth', 2.0);
  xlabel("Time [s]");
  ylabel("Displacement [m]");
  title(plot_title);
  legend("x", "y", "z");

  subplot(212);
  hold on;
  cam_euler = [];
  for i = 1:rows(data)
    q = data(i, 5:8);
    cam_euler = [cam_euler; quat2euler(q)'];
  endfor
  plot(data(:, 1) * 1e-9, cam_euler(:, 1), 'r-', 'linewidth', 2.0);
  plot(data(:, 1) * 1e-9, cam_euler(:, 2), 'g-', 'linewidth', 2.0);
  plot(data(:, 1) * 1e-9, cam_euler(:, 3), 'b-', 'linewidth', 2.0);
  xlabel("Time [s]");
  ylabel("Rotation [deg]");
  legend("x", "y", "z");
endfunction

function plot_imu_data(fig_id, plot_title, data)
  figure(fig_id);

  subplot(211);
  hold on;
  plot(data(:, 1) * 1e-9, data(:, 2), 'r-', 'linewidth', 2.0);
  plot(data(:, 1) * 1e-9, data(:, 3), 'g-', 'linewidth', 2.0);
  plot(data(:, 1) * 1e-9, data(:, 4), 'b-', 'linewidth', 2.0);
  xlabel("Time [s]");
  ylabel("Accel [m/s]");
  title(plot_title);
  legend("x", "y", "z");

  subplot(212);
  hold on;
  plot(data(:, 1) * 1e-9, data(:, 5), 'r-', 'linewidth', 2.0);
  plot(data(:, 1) * 1e-9, data(:, 6), 'g-', 'linewidth', 2.0);
  plot(data(:, 1) * 1e-9, data(:, 7), 'b-', 'linewidth', 2.0);
  xlabel("Time [s]");
  ylabel("Gyro [rad/s]");
  legend("x", "y", "z");
endfunction

% 3D plot
plot_3d(1, "3D Plot", features, cam_poses);
plot_poses(2, "Camera Poses", cam_poses);
plot_imu_data(3, "IMU data", imu_data);
plot_poses(4, "Interpolated Poses", imu_poses);

ginput();
