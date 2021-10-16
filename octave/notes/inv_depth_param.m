addpath(genpath("proto"));

% Setup cam0
cam_idx = 0;
image_width = 640;
image_height = 480;
resolution = [image_width; image_height];
fov = 60.0;
fx = focal_length(image_width, fov);
fy = focal_length(image_height, fov);
cx = image_width / 2;
cy = image_height / 2;
proj_params = [fx; fy; cx; cy];
dist_params = [-0.01; 0.01; 1e-4; 1e-4];
camera = pinhole_radtan4_init(cam_idx, resolution, proj_params, dist_params);

% Camera pose in world frame
C_WC = euler321([-pi / 2, 0.0, -pi / 2]);
r_WC = [0.01; 0.02; 0.03];
T_WC = tf(C_WC, r_WC);

% Point in camera frame
p_W = [10.0; 0.01; 0.03];
p_C = tf_point(inv(T_WC), p_W);
z = pinhole_radtan4_project(proj_params, zeros(4, 1), p_C)

% Image pixel measurement to 3D ray in world frame
x = [(z(1) - cx) / fx; (z(2) - cy) / fy; 1.0];
h_W = C_WC * x;

% Form inverse-depth parameterization
[theta, phi] = point2bearing(h_W);
rho = 1.0 / p_C(3);
param = [r_WC; theta; phi; rho];

% Convert inverse depth parameterization to 3D world point
p_W = idp2point(param)
