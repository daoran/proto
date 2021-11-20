function J_params = equi4_params_jacobian(dist_params, p)
  assert(length(dist_params) == 4);
  assert(length(p) == 2);

  % Distortion parameters
  k1 = dist_params(1);
  k2 = dist_params(2);
  k3 = dist_params(3);
  k4 = dist_params(4);

  % Jacobian
  x = p(1);
  y = p(2);
  r = sqrt(x**2 + y**2);
  th = atan(r);

  J_params = zeros(2, 4);

  J_params(1, 1) = x * th**3 / r;
  J_params(1, 2) = x * th**5 / r;
  J_params(1, 3) = x * th**7 / r;
  J_params(1, 4) = x * th**9 / r;

  J_params(2, 1) = y * th**3 / r;
  J_params(2, 2) = y * th**5 / r;
  J_params(2, 3) = y * th**7 / r;
  J_params(2, 4) = y * th**9 / r;
endfunction
