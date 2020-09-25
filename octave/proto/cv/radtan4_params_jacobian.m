function J_params = radtan4_params_jacobian(dist_params, p)
  assert(length(dist_params) == 4);
  assert(length(p) == 2);

  % Distortion parameters
  k1 = dist_params(1);
  k2 = dist_params(2);
  p1 = dist_params(3);
  p2 = dist_params(4);

  % Point
  x = p(1);
  y = p(2);

  % Setup
  x2 = x * x;
  y2 = y * y;
  xy = x * y;
  r2 = x2 + y2;
  r4 = r2 * r2;

  % Params Jacobian
  J_params = zeros(2, 4);
  J_params(1, 1) = x * r2;
  J_params(1, 2) = x * r4;
  J_params(1, 3) = 2 * xy;
  J_params(1, 4) = 3 * x2 + y2;
  J_params(2, 1) = y * r2;
  J_params(2, 2) = y * r4;
  J_params(2, 3) = x2 + 3 * y2;
  J_params(2, 4) = 2 * xy;
endfunction
