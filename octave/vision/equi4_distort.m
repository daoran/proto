function [point_distorted, J] = equi4_distort(k1, k2, k3, k4, point)
  x = point(1);
  y = point(2);

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
  point_distorted = [x_dash, y_dash];

  th_r = 1.0 / (r * r + 1.0);
  thd_th = 1.0 + 3.0 * k1 * th2 + 5.0 * k2 * th4 + 7.0 * k3 * th6 + 9.0 * k4 * th8;
  s_r = thd_th * th_r / r - thd / (r * r);
  r_x = 1.0 / r * x;
  r_y = 1.0 / r * y;
  J = zeros(2, 2);
  J(1,1) = s + x * s_r * r_x;
  J(1,2) = x * s_r * r_y;
  J(2,1) = y * s_r * r_x;
  J(2,2) = s + y * s_r * r_y;
endfunction
