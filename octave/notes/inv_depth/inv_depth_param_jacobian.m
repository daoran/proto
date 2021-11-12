addpath(genpath("proto"));

step = 1e-8;
threshold = 1e-4;

function p_W = m(theta_i, phi_i)
  p_W = [cos(phi_i) * sin(theta_i);
         -sin(phi_i);
         cos(phi_i) * cos(theta_i)];
endfunction

function test_inv_depth_param_jac(step, threshold)
  C_WC = euler321([-pi / 2, 0.0, -pi / 2]);
  r_WC = [0.01; 0.02; 0.03];

  x_i = 0.0;
  y_i = 0.0;
  z_i = 0.0;
  rho_i = 1.0 / 10.0;
  theta_i = 0.1;
  phi_i = 0.2;

  p_W = [x_i; y_i; z_i] + 1 / rho_i * m(theta_i, phi_i);
  J_x_i = [1; 0; 0];
  J_y_i = [0; 1; 0];
  J_z_i = [0; 0; 1];
  J_theta_i = 1 / rho_i * [cos(phi_i) * cos(theta_i); 0.0; cos(phi_i) * -sin(theta_i)];
  J_phi_i = 1 / rho_i * [-sin(phi_i) * sin(theta_i); -cos(phi_i); -sin(phi_i) * cos(theta_i)];
  J_rho_i = -1 / rho_i**2 * m(theta_i, phi_i);
  J_inv_depth = [J_x_i, J_y_i, J_z_i, J_theta_i, J_phi_i, J_rho_i];

  % Numerical differentiation
  J_numdiff = zeros(3, 6);

  p_W_prime = [x_i + step; y_i; z_i] + 1 / rho_i * m(theta_i, phi_i);
  J_numdiff(1:3, 1) = (p_W_prime - p_W) / step;

  p_W_prime = [x_i; y_i + step; z_i] + 1 / rho_i * m(theta_i, phi_i);
  J_numdiff(1:3, 2) = (p_W_prime - p_W) / step;

  p_W_prime = [x_i; y_i; z_i + step] + 1 / rho_i * m(theta_i, phi_i);
  J_numdiff(1:3, 3) = (p_W_prime - p_W) / step;

  p_W_prime = [x_i; y_i; z_i] + 1 / (rho_i) * m(theta_i + step, phi_i);
  J_numdiff(1:3, 4) = (p_W_prime - p_W) / step;

  p_W_prime = [x_i; y_i; z_i] + 1 / (rho_i) * m(theta_i, phi_i + step);
  J_numdiff(1:3, 5) = (p_W_prime - p_W) / step;

  p_W_prime = [x_i; y_i; z_i] + 1 / (rho_i + step) * m(theta_i, phi_i);
  J_numdiff(1:3, 6) = (p_W_prime - p_W) / step;

  % J_inv_depth
  % J_numdiff
  check_jacobian("J_inv_depth", J_numdiff, J_inv_depth, threshold, true);
endfunction

function [z, J] = project(proj_params, dist_params, T_WC, idp_params)
  x_i = idp_params(1);
  y_i = idp_params(2);
  z_i = idp_params(3);
  theta = idp_params(4);
  phi = idp_params(5);
  rho = idp_params(6);

  p_W = [x_i; y_i; z_i] + 1 / rho * m(theta, phi);
  p_C = tf_point(inv(T_WC), p_W);
  z = pinhole_radtan4_project(proj_params, dist_params, p_C);
  Jh = pinhole_radtan4_project_jacobian(proj_params, dist_params, p_C);

  J_x_i = [1; 0; 0];
  J_y_i = [0; 1; 0];
  J_z_i = [0; 0; 1];
  J_theta = 1 / rho * [cos(phi) * cos(theta); 0.0; cos(phi) * -sin(theta)];
  J_phi = 1 / rho * [-sin(phi) * sin(theta); -cos(phi); -sin(phi) * cos(theta)];
  J_rho = -1 / rho**2 * m(theta, phi);
  J_params = [J_x_i, J_y_i, J_z_i, J_theta, J_phi, J_rho];

  C_WC = tf_rot(T_WC);
  J = Jh * C_WC' * J_params;
endfunction

function test_inv_depth_param_jac2(step, threshold)
  % Setup camera pose
  C_WC = euler321([-pi / 2, 0.0, -pi / 2]);
  r_WC = [0.01; 0.02; 0.03];
  T_WC = tf(C_WC, r_WC);

  % Setup camera
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

  % Inverse depth parameterization
  x_i = 0.1;
  y_i = 0.2;
  z_i = 0.3;
  rho = 1.0 / 10.0;
  theta = 0.4;
  phi = 0.3;
  idp_params = [x_i, y_i, z_i, theta, phi, rho];

  % Project and get jacobian
  [z, J_inv_depth] = project(proj_params, dist_params, T_WC, idp_params);

  % Numerical differentiation
  J_numdiff = zeros(2, 6);
  for i = 1:6
    idp_params_fwd = idp_params;
    idp_params_fwd(i) += 0.5 * step;
    [z_fwd, _] = project(proj_params, dist_params, T_WC, idp_params_fwd);

    idp_params_bwd = idp_params;
    idp_params_bwd(i) -= 0.5 * step;
    [z_bwd, _] = project(proj_params, dist_params, T_WC, idp_params_bwd);

    J_numdiff(1:2, i) = (z_fwd - z_bwd) / step;
  endfor

  % J_inv_depth
  % J_numdiff
  check_jacobian("J_inv_depth", J_numdiff, J_inv_depth, threshold, true);
endfunction

test_inv_depth_param_jac(step, threshold)
test_inv_depth_param_jac2(step, threshold)
