addpath(genpath("proto"));

% pkg load symbolic;
%
% % Setup symbols
% syms k1 k2 p1 p2;
% syms x y;
%
% x2 = x * x;
% y2 = y * y;
% r2 = x2 + y2;
% r4 = r2 * r2;
%
% % Radial distortion
% radial_factor = 1.0 + (k1 * r2) + (k2 * r4);
% x_dash = x * radial_factor;
% y_dash = y * radial_factor;
%
% % Tangential distortion
% xy = x * y;
% x_ddash = x_dash + (2.0 * p1 * xy + p2 * (r2 + 2.0 * x2));
% y_ddash = y_dash + (p1 * (r2 + 2.0 * y2) + 2.0 * p2 * xy);
%
% % Jacobian of radtan w.r.t input x, y
% J = jacobian([x_ddash, y_ddash], [k1, k2, p1, p2])
% char(J(1, 1))
% char(J(1, 2))
% char(J(1, 3))
% char(J(1, 4))
% char(J(2, 1))
% char(J(2, 2))
% char(J(2, 3))
% char(J(2, 4))


function retval = check_radtan4_param_jacobian(dist_params)
  % Radtan distort
  p = [unifrnd(-0.1, 0.1); unifrnd(-0.1, 0.1)];
  z = radtan4_distort(dist_params, p);

  % Jacobian w.r.t. dist_params
  J = radtan4_params_jacobian(dist_params, p);

  % Finite diff w.r.t dist_params
  fdiff = zeros(2, 4);
  step_size = 1.0e-8;

  % -- Perturb k1
  dist_params_diff = dist_params;
  dist_params_diff(1) = dist_params(1) + step_size;
  z_prime = radtan4_distort(dist_params_diff, p);
  fdiff(1:2, 1) = (z_prime - z) / step_size;

  % -- Perturb k2
  dist_params_diff = dist_params;
  dist_params_diff(2) = dist_params(2) + step_size;
  z_prime = radtan4_distort(dist_params_diff, p);
  fdiff(1:2, 2) = (z_prime - z) / step_size;

  % -- Perturb p1
  dist_params_diff = dist_params;
  dist_params_diff(3)  = dist_params(3) + step_size;
  z_prime = radtan4_distort(dist_params_diff, p);
  fdiff(1:2, 3) = (z_prime - z) / step_size;

  % -- Perturb p2
  dist_params_diff = dist_params;
  dist_params_diff(4) = dist_params(4) + step_size;
  z_prime = radtan4_distort(dist_params_diff, p);
  fdiff(1:2, 4) = (z_prime - z) / step_size;

  threshold = 1e-5;
  retval = check_jacobian("J_radnta4_params", fdiff, J, threshold);
endfunction

% Check jacobian
k1 = 0.01;
k2 = 0.001;
p1 = 0.01;
p2 = 0.001;
dist_params = [k1; k2; p1; p2];
assert(check_radtan4_param_jacobian(dist_params) == 0);
