addpath(genpath("proto"));
test_passed = true;

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
p_W = [10.0; 0.1; 0.2];

% Point in camera frame
p_C = tf_point(inv(T_WC), p_W);

% Normalize point
x = [p_C(1) / p_C(3);
     p_C(2) / p_C(3)];

% Project to image plane
z = [fx * x(1) + cx; fy * x(2) + cy];

% Pinhole point jacobian
J = pinhole_point_jacobian(proj_params);

% Perform numerical diff to obtain finite difference
step_size = 1e-6;
finite_diff = zeros(2, 2);
for i = 1:2
  x_diff = x;
  x_diff(i) += step_size;
  z_diff = [fx * x_diff(1) + cx; fy * x_diff(2) + cy];
  finite_diff(1:2, i) = (z_diff - z) / step_size;
endfor

% Compare numerical finite diff and analytical jacobian
% Make sure the difference between all entries are no large than the
% numerical diff step size
if any(all((finite_diff - J) > step_size)) == 1
  printf("Invalid Jacobian?\n")
  test_passed = false;
endif

if test_passed == false
  exit(-1);
endif
