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


function check_radtan4_point_jacobian(k1, k2, p1, p2)
  % Radtan distort
  p = [unifrnd(-0.1, 0.1); unifrnd(-0.1, 0.1)];
  z = radtan4_distort(k1, k2, p1, p2, p);
  J = radtan4_point_jacobian(k1, k2, p1, p2, p);

  % Jacobian w.r.t. point x and y
  x = p(1);
  y = p(2);

  J11 = k1*(x**2 + y**2) + k2*(x**2 + y**2)**2 + 2*p1*y + 6*p2*x + x*(2*k1*x + 4*k2*x*(x**2 + y**2)) + 1;
  J12 = 2*p1*x + 2*p2*y + y*(2*k1*x + 4*k2*x*(x**2 + y**2));
  J21 = 2*p1*x + 2*p2*y + x*(2*k1*y + 4*k2*y*(x**2 + y**2));
  J22 = k1*(x**2 + y**2) + k2*(x**2 + y**2)**2 + 6*p1*y + 2*p2*x + y*(2*k1*y + 4*k2*y*(x**2 + y**2)) + 1;
  dz__dp = [J11, J12; J21, J22];

  % Finite diff w.r.t input point (x, y)
  fdiff = zeros(2, 2);
  step_size = 1.0e-8;

  x_diff = x + step_size;
  z_prime = radtan4_distort(k1, k2, p1, p2, [x_diff, y]);
  fdiff(1:2, 1) = (z_prime - z) / step_size;

  y_diff = y + step_size;
  z_prime = radtan4_distort(k1, k2, p1, p2, [x, y_diff]);
  fdiff(1:2, 2) = (z_prime - z) / step_size;

  % Check jacobian
  threshold = 1e-5;
  retval = check_jacobian("dz__dp", fdiff, dz__dp, threshold);
endfunction

% Check jacobian
k1 = 0.01;
k2 = 0.001;
p1 = 0.01;
p2 = 0.001;
check_radtan4_point_jacobian(k1, k2, p1, p2);
