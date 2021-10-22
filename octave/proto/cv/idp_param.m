function param = idp_param(camera, T_WC, z)
  % Back project image pixel measurmeent to 3D ray
  proj_params = camera.param(1:4);
  dist_params = camera.param(5:8);
  x = camera.backproject(proj_params, dist_params, z);

  % Convert 3D ray from camera frame to world frame
  r_WC = tf_trans(T_WC);
  C_WC = tf_rot(T_WC);
  h_W = C_WC * x;

  % Obtain bearing (theta, phi) and inverse depth (rho)
  theta = atan2(h_W(1), h_W(3));
  phi = atan2(-h_W(2), sqrt(h_W(1) * h_W(1) + h_W(3) * h_W(3)));
  rho = 0.1;
  % sigma_rho = 0.5; % variance of inverse depth

  % Form inverse depth parameter
  param = [r_WC; theta; phi; rho];
endfunction
