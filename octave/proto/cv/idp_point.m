function p_W = idp_point(param)
  % Camera position in world frame
  x = param(1);
  y = param(2);
  z = param(3);
  r_WC = [x; y; z];

  % Bearing angle (theta, phi) and inverse depth (rho)
  theta = param(4);
  phi = param(5);
  rho = param(6);

  % Convert bearing to 3D ray from camera frame
  m = [cos(phi) * sin(theta);
       -sin(phi);
       cos(phi) * cos(theta)];

  % Form 3D point in world frame
  p_W = r_WC + (1.0 / rho) * m;
endfunction
