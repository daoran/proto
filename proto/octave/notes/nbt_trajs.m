graphics_toolkit("fltk");

function hp = homogeneous(p)
  hp = [p; ones(1, columns(p))];
endfunction

function y = skew(x)
  y = [0, -x(3), x(2);
       x(3), 0, -x(1);
       -x(2), x(1), 0];
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

function C = tf_rot(tf)
  C = tf(1:3, 1:3);
endfunction

function retval = tf_point(T, point)
  assert(all(size(point) == [3, 1]));
  hpoint = [point; 1.0];
  retval = (T * hpoint)(1:3);
endfunction

function poses = load_poses(csv_file)
  csv = csvread(csv_file);
  poses = {};
  for i = 1:rows(csv)
    q = csv(i, 1:4)';
    r = csv(i, 5:7)';
    poses{i} = tf(q, r);
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

function p_distorted = radtan4_distort(k1, k2, p1, p2, p)
  % Point
  x = p(1);
  y = p(2);

  % Apply radial distortion
  x2 = x * x;
  y2 = y * y;
  r2 = x2 + y2;
  r4 = r2 * r2;
  radial_factor = 1.0 + (k1 * r2) + (k2 * r4);
  x_dash = x * radial_factor;
  y_dash = y * radial_factor;

  % Apply tangential distortion
  xy = x * y;
  x_ddash = x_dash + (2.0 * p1 * xy + p2 * (r2 + 2.0 * x2));
  y_ddash = y_dash + (p1 * (r2 + 2.0 * y2) + 2.0 * p2 * xy);
  p_distorted = [x_ddash; y_ddash];
endfunction

function J_params = radtan4_params_jacobian(k1, k2, p1, p2, p)
  % Point
  x = p(1);
  y = p(2);

  % Setup
  x2 = x * x;
  y2 = y * y;
  xy = x * y;
  r2 = x2 + y2;
  r4 = r2 * r2;

  % Params Jacobian
  J_params = zeros(2, 4);
  J_params(1, 1) = x * r2;
  J_params(1, 2) = x * r4;
  J_params(1, 3) = 2 * xy;
  J_params(1, 4) = 3 * x2 + y2;
  J_params(2, 1) = y * r2;
  J_params(2, 2) = y * r4;
  J_params(2, 3) = x2 + 3 * y2;
  J_params(2, 4) = 2 * xy;
endfunction

function J_point = radtan4_point_jacobian(k1, k2, p1, p2, p)
  % Point
  x = p(1);
  y = p(2);

  % Apply radial distortion
  x2 = x * x;
  y2 = y * y;
  r2 = x2 + y2;
  r4 = r2 * r2;

  % Point Jacobian
  % Let u = [x; y] normalized point
  % Let u' be the distorted u
  % The jacobian of u' w.r.t. u (or du'/du) is:
  J_point = zeros(2, 2);
  J_point(1, 1) = k1 * r2 + k2 * r4 + 2 * p1 * y + 6 * p2 * x + x *(2 * k1 * x + 4 * k2 *x * r2) + 1;
  J_point(2, 1) = 2 * p1 * x + 2 * p2 * y + y * (2 * k1 * x + 4 * k2 * x * r2);
  J_point(1, 2) = J_point(2, 1);
  J_point(2, 2) = k1 * r2 + k2 * r4 + 6 * p1 * y + 2 * p2 * x + y * (2 * k1 * y + 4 * k2 * y * r2) + 1;
  % Above is generated using sympy
endfunction

% function form_test_grid()
%   % Test grid properties
%   grid_rows = 5;
%   grid_cols = 5;
%   grid_depth = 5.0;
%   dx = 752 / (grid_cols + 1);
%   dy = 480 / (grid_rows + 1);
%
%   % Form test grid
%   kp_x = 0;
%   kp_y = 0;
%   object_points = [];
%
%   for (int i = 1; i < (grid_cols + 1); i++) {
%     kp_y += dy;
%     for (int j = 1; j < (grid_rows + 1); j++) {
%       kp_x += dx;
%
%       % Keypoint
%       const vec2_t kp{kp_x, kp_y};
%       keypoints.push_back(kp);
%
%       % Object point
%       Eigen::Vector3d ray;
%       cam.backProject(kp, &ray);
%       object_points.push_back(ray * grid_depth);
%     endfor
%     kp_x = 0;
%   endfor
% endfunction

function entropy = eval_nbt(sensor_poses, T_SC, cam_params, calib_covar)
  entropy = 0;
  p_W = [10; 0; 0];

  for i = 1:length(sensor_poses)
    % Setup
    T_WS = sensor_poses{i};
    C_WS = tf_rot(T_WS);
    C_SC = tf_rot(T_SC);
    T_WC = T_WS * T_SC;
    T_CW = T_WC';
    p_C = tf_point(T_CW, p_W);
    % p_C = [0; 0; 10.0];

    % Project and distort point p_C
    fx = cam_params(1);
    fy = cam_params(2);
    k1 = cam_params(5);
    k2 = cam_params(6);
    p1 = cam_params(7);
    p2 = cam_params(8);
    x = [p_C(1) / p_C(3); p_C(2) / p_C(3)];
    x_dist = radtan4_distort(k1, k2, p1, p2, x);  % Distort point

    % Project-distort-scale and center jacobian
    J_proj = [1 / p_C(3), 0, -p_C(1) / p_C(3)**2;
              0, 1 / p_C(3), -p_C(2) / p_C(3)**2];
    J_dist = radtan4_point_jacobian(k1, k2, p1, p2, x);
    J_point = [fx, 0; 0, fy];
    J_h = J_point * J_dist * J_proj;

    % Sensor pose Jacobian
    p_S = tf_point(T_SC, p_C);
    dp_C__dp_W = C_SC' * C_WS';
    dp_W__dr_WS = eye(3);
    dp_W__dtheta = -skew(C_WS * p_S);
    J_sensor_pose = zeros(2, 6);
    J_sensor_pose(1:2, 1:3) = J_h * dp_C__dp_W * dp_W__dr_WS;
    J_sensor_pose(1:2, 4:6) = J_h * dp_C__dp_W * dp_W__dtheta;

    % Sensor-camera Jacobian
    dp_C__dp_S = C_SC';
    dp_S__dr_SC = eye(3);
    dp_S__dtheta = -skew(C_SC * p_C);
    J_sensor_camera = zeros(2, 6);
    J_sensor_camera(1:2, 1:3) = J_h * dp_C__dp_S * dp_S__dr_SC;
    J_sensor_camera(1:2, 4:6) = J_h * dp_C__dp_S * dp_S__dtheta;

    % Camera parameters Jacobian
    J_cam_params = zeros(2, 8);
    J_cam_params(1:2, 1:4) = [x_dist(1), 0, 1, 0;
                              0, x_dist(2), 0, 1];
    J_dist_params = radtan4_params_jacobian(k1, k2, p1, p2, x);
    J_cam_params(1:2, 5:8) = J_point * J_dist_params;

    % First order error propagation
    J_x = [J_sensor_pose, J_sensor_camera, J_cam_params];
    sensor_pose_var = [1e-5; 1e-5; 1e-5; 1e-5; 1e-5; 1e-5];
    sensor_camera_var = diag(calib_covar)(1:6);
    cam_params_var = diag(calib_covar)(end-7:end);
    covar_x = diag([sensor_pose_var; sensor_camera_var; cam_params_var]);
    covar_y = (J_x * covar_x * J_x');

    entropy += trace(covar_y);
  endfor
endfunction


% MAIN
% -- Load data
T_WS_1 = load_poses("/tmp/T_WS-1.csv");
T_WS_2 = load_poses("/tmp/T_WS-2.csv");
T_WS_3 = load_poses("/tmp/T_WS-3.csv");
T_WS_4 = load_poses("/tmp/T_WS-4.csv");
T_WS_5 = load_poses("/tmp/T_WS-5.csv");
T_WS_6 = load_poses("/tmp/T_WS-6.csv");
T_WS_7 = load_poses("/tmp/T_WS-7.csv");
T_WS_8 = load_poses("/tmp/T_WS-8.csv");
calib_covar = csvread("/tmp/calib_covar.csv");
cam_params = csvread("/tmp/cam_params.csv");
T_WF = csvread("/tmp/T_WF.csv");
T_SC = csvread("/tmp/T_SC.csv");
p_FFi = csvread("/tmp/object_points.csv")';
p_WFi = (T_WF * [p_FFi; ones(1, columns(p_FFi))])(1:3, 1:end);

entropy = eval_nbt(T_WS_1, T_SC, cam_params, calib_covar)
entropy = eval_nbt(T_WS_2, T_SC, cam_params, calib_covar)
entropy = eval_nbt(T_WS_3, T_SC, cam_params, calib_covar)
entropy = eval_nbt(T_WS_4, T_SC, cam_params, calib_covar)
entropy = eval_nbt(T_WS_5, T_SC, cam_params, calib_covar)
entropy = eval_nbt(T_WS_6, T_SC, cam_params, calib_covar)
entropy = eval_nbt(T_WS_7, T_SC, cam_params, calib_covar)
entropy = eval_nbt(T_WS_8, T_SC, cam_params, calib_covar)

% % -- Visualize Next-Best Trajectory
% figure(1);
% hold on;
% for i = 1:columns(p_WFi)
%   x = p_WFi(1, i);
%   y = p_WFi(2, i);
%   z = p_WFi(3, i);
%   scatter3(x, y, z, "r");
% endfor
% for i = 1:5:length(T_WS_1) draw_frame(T_WS_1{i}, 0.1); endfor
% for i = 1:5:length(T_WS_2) draw_frame(T_WS_2{i}, 0.1); endfor
% for i = 1:5:length(T_WS_3) draw_frame(T_WS_3{i}, 0.1); endfor
% for i = 1:5:length(T_WS_4) draw_frame(T_WS_4{i}, 0.1); endfor
% for i = 1:5:length(T_WS_5) draw_frame(T_WS_5{i}, 0.1); endfor
% for i = 1:5:length(T_WS_6) draw_frame(T_WS_6{i}, 0.1); endfor
% for i = 1:5:length(T_WS_7) draw_frame(T_WS_7{i}, 0.1); endfor
% for i = 1:5:length(T_WS_8) draw_frame(T_WS_8{i}, 0.1); endfor
% view(3);
% xlabel("x");
% ylabel("y");
% zlabel("z");
% axis 'equal';
% ginput();
