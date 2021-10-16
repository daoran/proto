function p_W = idp2point(param)
  x = param(1);
  y = param(2);
  z = param(3);
  r_WC = [x; y; z];

  theta = param(4);
  phi = param(5);
  rho = param(6);

  m = [cos(phi) * sin(theta);
       -sin(phi);
       cos(phi) * cos(theta)];

  p_W = r_WC + (1.0 / rho) * m;
endfunction
