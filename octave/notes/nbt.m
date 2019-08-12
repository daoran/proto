addpath(genpath("proto"));
graphics_toolkit("fltk");

function points = points_inview(calib_target, T_WT, cam, T_WC)
  hp_T = homogeneous(calib_target.object_points);
  hp_C = inv(T_WC) * T_WT * hp_T;
  p_C = dehomogeneous(hp_C);

  points = [];
  for i = 1:length(p_C)
    p = p_C(1:3, i);

    % Project
    x = p(1) / p(3);
    y = p(2) / p(3);
    p = [x; y];

    % Distort
    p_d = radtan4_distort(cam.k1, cam.k2, cam.p1, cam.p2, p);
    pixel_x = cam.fx * p_d(1) + cam.cx;
    pixel_y = cam.fy * p_d(2) + cam.cy;

    if pixel_x < cam.resolution(1) && pixel_y < cam.resolution(2)
      p_W = dehomogeneous(T_WC * [p_C(:, i); 1.0]);
      points = [points, p_W];
    end
  endfor
endfunction

function plot_camera_view(cam, T_WC, calib_target, T_WT)
  hp_T = homogeneous(calib_target.object_points);
  hp_C = inv(T_WC) * T_WT * hp_T;
  p_C = dehomogeneous(hp_C);

  points = [];
  for i = 1:length(p_C)
    p = p_C(1:3, i);

    % Project
    x = p(1) / p(3);
    y = p(2) / p(3);
    p = [x; y];

    % Distort
    p_d = radtan4_distort(cam.k1, cam.k2, cam.p1, cam.p2, p);
    pixel_x = cam.fx * p_d(1) + cam.cx;
    pixel_y = cam.fy * p_d(2) + cam.cy;

    if pixel_x < cam.resolution(1) && pixel_y < cam.resolution(2)
      plot(pixel_x, pixel_y, "rx", "linewidth", 2.0);
    end
  endfor

  axis("equal");
  xlabel("x [px]");
  ylabel("y [px]");
  xlim([0, cam.resolution(1)]);
  ylim([0, cam.resolution(2)]);
endfunction

function l_trajs = generate_linear_trajectories(calib_target, T_WT, T_TO)
  % Trajectory parameters
  circle_radius = calib_target.width / 2.0;
  theta_min = deg2rad(0.0);
  theta_max = deg2rad(360.0);

  % Trajectory start (origin)
  ho = [0.0; 0.0; 0.0; 1.0];
  ho_WH = T_WT * T_TO * ho;
  o_WH = ho_WH(1:3);

  % Pan trajectories
  l_trajs = {};
  for theta = linspace(theta_min, theta_max, 9)
    % Trajectory end (circle point)
    hp = [circle(circle_radius, theta); 0.0; 1.0];
    hp_WH = T_WT * T_TO * hp;
    p_WH = hp_WH(1:3);

    % Generate camera poses
    rpy = deg2rad([-90.0, 0.0, -90.0]);
    C_WC = euler321(rpy);
    T_WC0 = tf(C_WC, o_WH); % Calculate first camera pose
    T_WC1 = tf(C_WC, p_WH); % Calculate second camera pose

    traj = {T_WC0, T_WC1};
    l_trajs{end + 1} = traj;
  endfor

  % Forward backward
  % -- Trajectory end (circle point)
  z_TO = T_TO(3, 4);
  hp = ho + [0.0; 0.0; -0.5 * z_TO; 0.0];
  hp_WH = T_WT * T_TO * hp;
  p_WH = hp_WH(1:3);
  % -- Generate camera poses
  rpy = deg2rad([-90.0, 0.0, -90.0]);
  C_WC = euler321(rpy);
  T_WC0 = tf(C_WC, o_WH);  % Calculate first camera pose
  T_WC1 = tf(C_WC, p_WH);  % Calculate second camera pose
  traj = {T_WC0, T_WC1};
  l_trajs{end + 1} = traj;
endfunction

function s_trajs = generate_spherical_trajectories(calib_target, T_WT, T_TO, r_TTc)
  % Trajectory parameters
  rho = calib_target.width / 2.0;  % Sphere radius
  latitude_min = deg2rad(0.0);
  latitude_max = deg2rad(360.0);
  longitude_min = deg2rad(0.0);
  longitude_max = deg2rad(70.0);
  T_OJ = tf(eye(3), [0.0; 0.0; -rho]);

  s_trajs = {};
  for latitude = linspace(latitude_min, latitude_max, 9)
    traj = {};
    for longitude = linspace(longitude_min, longitude_max, 10)
      hr_TJ = T_TO * T_OJ * [sphere(rho, longitude, latitude); 1.0];
      r_TJ = dehomogeneous(hr_TJ);
      T_TC = lookat(r_TJ, r_TTc);
      traj{end + 1} = T_WT * T_TC;
    endfor

    s_trajs{end + 1} = traj;
  endfor
endfunction

% Setup camera
cam.resolution = [640; 480];
cam.fx = focal_length(cam.resolution(1), 90);
cam.fy = focal_length(cam.resolution(2), 75);
cam.cx = cam.resolution(1) / 2.0;
cam.cy = cam.resolution(2) / 2.0;
cam.k1 = 0.01;
cam.k2 = 0.001;
cam.p1 = 0.001;
cam.p2 = 0.001;

% Setup calibration target
calib_rows = 6;
calib_cols = 7;
calib_sq_size = 0.2;
calib_target = calib_target_init(calib_rows, calib_cols, calib_sq_size);

C_WT = euler321(deg2rad([90.0, 0.0, -90.0]));
T_WT = eye(4);
T_WT(1:3, 1:3) = C_WT;
T_WT(1:3, 4) = zeros(3, 1);

% Calculate target center in target frame
r_TTc = [calib_target.center; 0.0];

% Create trajectory origin infront relative to target
% Note: target center is bottom left corner
C_TO = eye(3);
z_TO = 1.0;  % Distance away from calibration target center
r_TO = [calib_target.center; z_TO];
T_TO = tf(C_TO, r_TO);

% Generate trajectories
l_trajs = generate_linear_trajectories(calib_target, T_WT, T_TO);
% s_trajs = generate_spherical_trajectories(calib_target, T_WT, T_TO, r_TTc);

figure();
hold on;
traj = l_trajs{1};
T_WC = traj{1};
T_WC(1:3, 4) = [-3.0; 0.0; 0.0];
plot_camera_view(cam, T_WC, calib_target, T_WT);
ginput();

% % Plot scene
% figure();
% hold on;
%
% for i = 1:length(l_trajs)
%   traj = l_trajs{i};
%   for j = 1:length(traj)
%     T_WC = traj{j};
%     points = points_inview(calib_target, T_WT, cam, T_WC)
%     draw_frame(T_WC, 0.1);
%   endfor
% endfor
%
% % for i = 1:length(s_trajs)
% %   traj = s_trajs{i};
% %   for j = 1:length(traj)
% %     draw_frame(traj{j}, 0.1);
% %   endfor
% % endfor
%
% calib_target_draw(calib_target, T_WT);
%
% view(3);
% xlabel("x [m]");
% ylabel("y [m]");
% zlabel("z [m]");
% axis "equal";
% ginput();
