addpath(genpath("proto"));
pkg load symbolic;

% syms P11 P12 P13 P14;
% syms P21 P22 P23 P24;
% syms P31 P32 P33 P34;
% syms Xx Xy Xz Xw;
% syms x y z;
%
% P = [P11, P12, P13, P14;
%      P21, P22, P23, P24;
%      P31, P32, P33, P34];
% X = [Xx; Xy; Xz; 1.0];
% z = [x; y; 1.0];
%
% cross(z, (P * X))

function p = linear_triangulation(z, z_dash, P, P_dash)
  % Image point from the first frame
  x = z(1);
  y = z(2);

  % Image point from the second frame
  x_dash = z_dash(1);
  y_dash = z_dash(2);

  % First three rows of P
  P1T = P(1, :);
  P2T = P(2, :);
  P3T = P(3, :);

  % First three rows of P_dash
  P1T_dash = P_dash(1, :);
  P2T_dash = P_dash(2, :);
  P3T_dash = P_dash(3, :);

  % Form the A matrix of AX = 0
  % A = [x * P3T - P1T;
  %      y * P3T - P2T;
  %      x_dash * P3T_dash - P1T_dash;
  %      y_dash * P3T_dash - P2T_dash];

  A = [y * P3T - P2T;
       x * P3T - P1T;
       y_dash * P3T_dash - P2T_dash;
       x_dash * P3T_dash - P1T_dash];

  % Use SVD to solve AX = 0
  [_, _, V] = svd(A' * A);
  hp = V(:, 4); % Get the best result from SVD (last column of V)

  hp = hp / hp(4);  % Normalize the homogeneous 3D point
  p = hp(1:3);      % Return only the first three components (x, y, z)
endfunction

% Setup camera
image_width = 640;
image_height = 480;
fov = 120.0;
fx = focal_length(image_width, fov);
fy = focal_length(image_height, fov);
cx = image_width / 2;
cy = image_height / 2;
proj_params = [fx, fy, cx, cy];
K = pinhole_K(proj_params);

% Setup camera pose T_WC0
rot = euler321(deg2rad([-90; 0; -90]));
trans = [0.0; 0.0; 0.0];
T_WC0 = tf(rot, trans);

% Setup camera pose T_WC1
rot = euler2quat(deg2rad([-90; 0; -90]));
trans = [0.1; 0.1; 0.0];
T_WC1 = tf(rot, trans);

% Setup projection matrices
T_C0W = inv(T_WC0);
C0 = tf_rot(T_C0W);
t0 = tf_trans(T_C0W);
P0 = K * [C0, t0];

T_C1W = inv(T_WC1);
C1 = tf_rot(T_C1W);
t1 = tf_trans(T_C1W);
P1 = K * [C1, t1];

% Setup 3D and 2D correspondance points
nb_points = 20;
landmark_points = [];
cam0_points = [];
cam1_points = [];

p_W = [2.0; normrnd(0.0, 1.0); normrnd(0.0, 1.0)];
z0 = pinhole_project(proj_params, T_WC0, p_W);
z1 = pinhole_project(proj_params, T_WC1, p_W);

p_W
p_W_est = linear_triangulation(z0, z1, P0, P1)
