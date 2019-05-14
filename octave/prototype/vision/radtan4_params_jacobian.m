function [J_params] = radtan4_params_jacobian(k1, k2, p1, p2, p)
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
