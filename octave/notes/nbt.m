addpath(genpath("proto"));

function p = pinhole_radtan4_project(cam, p_C)
  % Project
  x = p_C(1) / p_C(3);
  y = p_C(2) / p_C(3);
  p = [x; y];

  % Distort, scale and center
  p_d = radtan4_distort(cam.k1, cam.k2, cam.p1, cam.p2, p);
  pixel_x = cam.fx * p_d(1) + cam.cx;
  pixel_y = cam.fy * p_d(2) + cam.cy;

  p = [pixel_x; pixel_y];
endfunction

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

    % Distort, scale and center
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

    % Distort, scale and center
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

function T_TO = calc_calib_origin(calib_target, cam)
  % Standard pinhole camera model equation
  % x = f * X / Z
  % x / f * X = 1 / Z
  % Z = f * X / x

  % Calculate distance away from target center
  half_target_width = calib_target.width / 2.0;
  half_resolution_x = cam.resolution(1) / 2.0;
  scale = 0.6; % Want the target width to occupy 60% of image space at T_TO
  z_TO = cam.fx * half_target_width / (half_resolution_x * scale);

  % Form T_TO
  C_TO = eye(3);
  scale = calib_target.width * 1.0;
  r_TO = [calib_target.center; z_TO];
  T_TO = tf(C_TO, r_TO);
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
  rho = calib_target.width / 1.8;  % Sphere radius
  latitude_min = deg2rad(0.0);
  latitude_max = deg2rad(360.0);
  longitude_min = deg2rad(0.0);
  longitude_max = deg2rad(80.0);
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

function plot_camera_view_ltraj(l_trajs, cam, calib_target, T_WT)
  figure("visible", "off");
  plot_idx = 1;
  for i = 1:length(l_trajs)
    traj = l_trajs{i};
    for j = 1:length(traj)
      T_WC = traj{j};
      clf;
      hold on;
      printf("processing frame %d\n", plot_idx);
      plot_camera_view(cam, T_WC, calib_target, T_WT);
      fname = sprintf("/tmp/ltraj-%02d.png", plot_idx);
      plot_idx++;
      print("-dpng", "-r100", fname);
    endfor
  endfor
  cmd = sprintf("ffmpeg -y -framerate 2 -i /%s/ltraj-%%02d.png -vf scale=1080:-1 ltraj.mp4", "tmp");
  system(cmd)
endfunction

function plot_camera_view_straj(s_trajs, cam, calib_target, T_WT)
  figure("visible", "off");
  plot_idx = 1;
  for i = 1:length(s_trajs)
    traj = s_trajs{i};
    for j = 1:length(traj)
      T_WC = traj{j};
      clf;
      hold on;
      printf("processing frame %d\n", plot_idx);
      plot_camera_view(cam, T_WC, calib_target, T_WT);
      fname = sprintf("/tmp/straj-%02d.png", plot_idx);
      plot_idx++;
      print("-dpng", "-r100", fname);
    endfor
  endfor
  cmd = sprintf("ffmpeg -y -framerate 5 -i /%s/straj-%%02d.png -vf scale=1080:-1 straj.mp4", "tmp");
  system(cmd)
endfunction

function plot_l_trajs(l_trajs, calib_target, T_WT)
  figure();
  hold on;

  for i = 1:length(l_trajs)
    traj = l_trajs{i};
    for j = 1:length(traj)
      T_WC = traj{j};
      draw_frame(T_WC, 0.1);
    endfor
  endfor
  calib_target_draw(calib_target, T_WT);

  view(3);
  xlabel("x [m]");
  ylabel("y [m]");
  zlabel("z [m]");
  axis "equal";
endfunction

function plot_s_trajs(s_trajs, calib_target, T_WT)
  figure();
  hold on;

  for i = 1:length(s_trajs)
    traj = s_trajs{i};
    for j = 1:length(traj)
      T_WC = traj{j};
      draw_frame(T_WC, 0.1);
    endfor
  endfor
  calib_target_draw(calib_target, T_WT);

  view(3);
  xlabel("x [m]");
  ylabel("y [m]");
  zlabel("z [m]");
  axis "equal";
endfunction

function plot_scene(l_trajs, s_trajs, calib_target, T_WT)
  figure();
  hold on;

  for i = 1:length(l_trajs)
    traj = l_trajs{i};
    for j = 1:length(traj)
      T_WC = traj{j};
      draw_frame(T_WC, 0.1);
    endfor
  endfor

  for i = 1:length(s_trajs)
    traj = s_trajs{i};
    for j = 1:length(traj)
      draw_frame(traj{j}, 0.1);
    endfor
  endfor

  calib_target_draw(calib_target, T_WT);

  view(3);
  xlabel("x [m]");
  ylabel("y [m]");
  zlabel("z [m]");
  axis "equal";
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
T_TO = calc_calib_origin(calib_target, cam);


% [calib_target.width; 0.0; z_TO];                         % Bottom right
% [calib_target.width; calib_target.height; z_TO];         % Top right
% [0.0; calib_target.height; z_TO];                        % Top left
% [calib_target.center(1); calib_target.center(2); z_TO];  % Center

% graphics_toolkit("fltk");
% figure();
% hold on;
%
% % T_WT = tf(eye(3), zeros(3, 1));
%
% % C_WC = euler321(deg2rad([-90.0, 0.0, -90.0]));
% % z_TO = T_TO(3, 4);
% % T_WC = tf(C_WC, tf_trans(T_WT * T_TO));
% % T_CT = inv(T_WC) * T_WT;
% % T_TC = inv(T_CT);
%
% draw_frame(T_WC);
% calib_target_draw(calib_target, T_WT);
%
% view(3);
% axis("equal");
% xlabel("x [m]");
% ylabel("y [m]");
% zlabel("z [m]");
% ginput();

% Generate trajectories
l_trajs = generate_linear_trajectories(calib_target, T_WT, T_TO);
s_trajs = generate_spherical_trajectories(calib_target, T_WT, T_TO, r_TTc);

% Plot camera view during the trajectories
% graphics_toolkit("gnuplot");
% plot_camera_view_ltraj(l_trajs, cam, calib_target, T_WT);
% plot_camera_view_straj(s_trajs, cam, calib_target, T_WT);

% Plot scene
graphics_toolkit("fltk");
% plot_scene(l_trajs, s_trajs, calib_target, T_WT);
plot_s_trajs(s_trajs, calib_target, T_WT);
ginput();
