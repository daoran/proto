addpath(genpath("prototype"));
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
J = jacobian([x_ddash, y_ddash], [k1, k2, p1, p2])
char(J(1, 1))
char(J(1, 2))
char(J(1, 3))
char(J(1, 4))
char(J(2, 1))
char(J(2, 2))
char(J(2, 3))
char(J(2, 4))


function check_radtan4_param_jacobian(k1, k2, p1, p2)
  % Radtan distort
  p = [unifrnd(-0.1, 0.1); unifrnd(-0.1, 0.1)];
  [z, _] = radtan4_distort(k1, k2, p1, p2, p);

  % Jacobian w.r.t. k1, k2, p1, p2
  x = p(1);
  y = p(2);
  dz__dradtan = [
    x*(x**2 + y**2), x*(x**2 + y**2)**2, 2*x*y, 3*x**2 + y**2;
    y*(x**2 + y**2), y*(x**2 + y**2)**2, x**2 + 3*y**2, 2*x*y
  ];

  % Finite diff w.r.t k1, k2, p1, p2
  fdiff = zeros(2, 4);
  step_size = 1.0e-8;

  k1_diff = k1 + step_size;
  [z_prime, _] = radtan4_distort(k1_diff, k2, p1, p2, p);
  fdiff(1:2, 1) = (z_prime - z) / step_size;

  k2_diff = k2 + step_size;
  [z_prime, _] = radtan4_distort(k1, k2_diff, p1, p2, p);
  fdiff(1:2, 2) = (z_prime - z) / step_size;

  p1_diff = p1 + step_size;
  [z_prime, _] = radtan4_distort(k1, k2, p1_diff, p2, p);
  fdiff(1:2, 3) = (z_prime - z) / step_size;

  p2_diff = p2 + step_size;
  [z_prime, _] = radtan4_distort(k1, k2, p1, p2_diff, p);
  fdiff(1:2, 4) = (z_prime - z) / step_size;

  threshold = 1e-5;
  retval = check_jacobian("dz__dradtan", fdiff, dz__dradtan, threshold);
endfunction

% Check jacobian
k1 = 0.01;
k2 = 0.001;
p1 = 0.01;
p2 = 0.001;
check_radtan4_param_jacobian(k1, k2, p1, p2);
