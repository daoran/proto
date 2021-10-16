addpath(genpath("proto"));

step = 1e-8;
C_WC = euler321([-pi / 2, 0.0, -pi / 2]);
r_WC = [0.01; 0.02; 0.03];

x_i = 0.0;
y_i = 0.0;
z_i = 0.0;
rho_i = 1.0 / 10.0;
theta_i = 0.1;
phi_i = 0.2;

function p_W = m(theta_i, phi_i)
  p_W = [cos(phi_i) * sin(theta_i);
         -sin(phi_i);
         cos(phi_i) * cos(theta_i)];
endfunction

p_W = [x_i; y_i; z_i] + 1 / rho_i * m(theta_i, phi_i)

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

J_inv_depth
J_numdiff
