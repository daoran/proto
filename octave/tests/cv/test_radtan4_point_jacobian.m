addpath(genpath("proto"));
pkg load symbolic;

% Setup symbols
syms k1 k2 p1 p2;
syms x y;

x2 = x * x;
y2 = y * y;
xy = x * y;
r2 = x2 + y2;
r4 = r2 * r2;

% Radial distortion
radial_factor = 1.0 + (k1 * r2) + (k2 * r4);
x_dash = x * radial_factor;
y_dash = y * radial_factor;

% Tangential distortion
xy = x * y;
x_ddash = x_dash + (2.0 * p1 * xy + p2 * (r2 + 2.0 * x2));
y_ddash = y_dash + (p1 * (r2 + 2.0 * y2) + 2.0 * p2 * xy);

% Jacobian of radtan w.r.t input x, y
J = jacobian([x_ddash, y_ddash], [x, y])
% char(J(1, 1))
% char(J(2, 1))
% char(J(1, 2))
% char(J(2, 2))


function check_radtan4_point_jacobian(dist_params)
  % Radtan distort
  p = [unifrnd(-0.1, 0.1); unifrnd(-0.1, 0.1)];
  z = radtan4_distort(dist_params, p);
  J = radtan4_point_jacobian(dist_params, p);

  % Jacobian w.r.t. point x and y
  J_point = radtan4_point_jacobian(dist_params, p);

  % Finite diff w.r.t input point (x, y)
  fdiff = zeros(2, 2);
  step_size = 1.0e-8;

  p_diff = p;
  p_diff(1) = p(1) + step_size;
  z_prime = radtan4_distort(dist_params, p_diff);
  fdiff(1:2, 1) = (z_prime - z) / step_size;

  p_diff = p;
  p_diff(2) = p(2) + step_size;
  z_prime = radtan4_distort(dist_params, p_diff);
  fdiff(1:2, 2) = (z_prime - z) / step_size;

  % Check jacobian
  threshold = 1e-5;
  retval = check_jacobian("J_radtan4_point", fdiff, J_point, threshold);
endfunction

% Check jacobian
k1 = 0.01;
k2 = 0.001;
p1 = 0.01;
p2 = 0.001;
dist_params = [k1; k2; p1; p2];
check_radtan4_point_jacobian(dist_params);
