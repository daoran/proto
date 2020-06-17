function J_point = radtan4_point_jacobian(k1, k2, p1, p2, p)
  % Point
  x = p(1);
  y = p(2);

  % Apply radial distortion
  x2 = x * x;
  y2 = y * y;
  r2 = x2 + y2;
  r4 = r2 * r2;

  % Point Jacobian
  % Let u = [x; y] normalized point
  % Let u' be the distorted u
  % The jacobian of u' w.r.t. u (or du'/du) is:
  J_point = zeros(2, 2);
  J_point(1, 1) = k1 * r2 + k2 * r4 + 2 * p1 * y + 6 * p2 * x + x *(2 * k1 * x + 4 * k2 *x * r2) + 1;
  J_point(2, 1) = 2 * p1 * x + 2 * p2 * y + y * (2 * k1 * x + 4 * k2 * x * r2);
  J_point(1, 2) = J_point(2, 1);
  J_point(2, 2) = k1 * r2 + k2 * r4 + 6 * p1 * y + 2 * p2 * x + y * (2 * k1 * y + 4 * k2 * y * r2) + 1;
  % Above is generated using sympy
endfunction
