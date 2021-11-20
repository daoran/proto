function p_distorted = equi4_distort(dist_params, p)
  assert(length(dist_params) == 4);
  assert(length(p) == 2);

  % Distortion parameters
  k1 = dist_params(1);
  k2 = dist_params(2);
  k3 = dist_params(3);
  k4 = dist_params(4);

  % Distort
  x = p(1);
  y = p(2);

  r = sqrt(x * x + y * y);
  th = atan(r);
  th2 = th * th;
  th4 = th2 * th2;
  th6 = th4 * th2;
  th8 = th4 * th4;
  thd = th * (1.0 + k1 * th2 + k2 * th4 + k3 * th6 + k4 * th8);
  s = thd / r;
  x_dash = s * x;
  y_dash = s * y;
  p_distorted = [x_dash; y_dash];
endfunction
