function p_distorted = radtan4_distort(dist_params, p)
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

  % Apply radial distortion
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
endfunction
