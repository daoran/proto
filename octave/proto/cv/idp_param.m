function param = idp_param(camera, T_WC, z)
  % Extract projection parameters
  fx = camera.proj_params(0);
  fy = camera.proj_params(1);
  cx = camera.proj_params(2);
  cy = camera.proj_params(3);

  % Convert image pixel coordinates to normalized retinal coordintes
  x = [(z(1) - cx) / fx; (z(2) - cy) / fy; 1.0];

  % Convert 3D ray from camera frame to world frame
  r_WC = tf_trans(T_WC);
  C_WC = tf_rot(T_WC);
  h_W = C_WC * x;

  % Obtain bearing (theta, phi) and inverse depth (rho)
  theta = atan2(h_W(1), h_W(3));
  phi = atan2(-h_W(2), sqrt(h_W(1) * h_W(1) + h_W(3) * h_W(3)));
  rho = 1.0;

  % Form inverse depth parameter
  param = [r_WC; theta; phi; rho];
endfunction
