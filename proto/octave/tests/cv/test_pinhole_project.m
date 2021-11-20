addpath(genpath("proto"));

% Camera
img_w = 640;
img_h = 480;
fx = 320.0;
fy = 320.0;
cx = img_w / 2.0;
cy = img_h / 2.0;
proj_params = [fx, fy, cx, cy];

% Camera pose in world frame
C_WC = euler321([-pi / 2, 0.0, -pi / 2]);
r_WC = [0.0; 0.0; 0.0];
T_WC = tf(C_WC, r_WC);

% 3D World point
p_W = [10.0; 0.0; 0.0];

% Pinhole project
z = pinhole_project(proj_params, T_WC, p_W);

assert(abs(z(1) - 320.0) < 1e-8);
assert(abs(z(2) - 240.0) < 1e-8);
