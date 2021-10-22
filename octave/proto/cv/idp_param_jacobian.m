function J_param = idp_param_jacobian(param)
  x = param(1);
  y = param(2);
  z = param(3);
  theta = param(4);
  phi = param(5);
  rho = param(6);
  p_W = [cos(phi) * sin(theta); -sin(phi); cos(phi) * cos(theta)];

  J_x = [1; 0; 0];
  J_y = [0; 1; 0];
  J_z = [0; 0; 1];
  J_theta = 1 / rho * [cos(phi) * cos(theta); 0.0; cos(phi) * -sin(theta)];
  J_phi = 1 / rho * [-sin(phi) * sin(theta); -cos(phi); -sin(phi) * cos(theta)];
  J_rho = -1 / rho**2 * p_W;
  J_param = [J_x, J_y, J_z, J_theta, J_phi, J_rho];
endfunction
