function [p_distorted, J] = radtan4_distort(k1, k2, p1, p2, p)
  % Apply radial distortion
  x = p(1);
  y = p(2);
  x2 = x * x;
  y2 = y * y;
  r2 = x2 + y2;
  r4 = r2 * r2;
  radial_factor = 1.0 + (k1 * r2) + (k2 * r4);
  x_dash = x * radial_factor;
  y_dash = y * radial_factor;

	% Apply tangential distortion
	xy = x * y;
	x_ddash = x_dash + (2.0 * p1 * xy + p2 * (r2 + 2.0 * x2));
	y_ddash = y_dash + (p1 * (r2 + 2.0 * y2) + 2.0 * p2 * xy);
  p_distorted = [x_ddash; y_ddash];


	% Let u = [x; y] normalized point
	% Let u' be the distorted u
  % The jacobian of u' w.r.t. u (or du'/du) is:
  J = zeros(2, 2);
	J(1, 1) = k1 * r2 + k2 * r4 + 2 * p1 * y + 6 * p2 * x + x *(2 * k1 * x + 4 * k2 *x * r2) + 1;
	J(2, 1) = 2 *p1 * x + 2 * p2 * y + y * (2 * k1 * x + 4 * k2 * x * r2);
	J(1, 2) = J(2, 1);
	J(2, 2) = k1 * r2 + k2 * r4 + 6 * p1 * y + 2 * p2 * x + y * (2 * k1 * y + 4 * k2 * y * r2) + 1;
	% Above is generated using sympy
endfunction
