function [J_point] = equi4_point_jacobian(k1, k2, k3, k4, p)
  x = p(1);
  y = p(2);
  r = sqrt(x**2 + y**2);

  th = atan(r);
  th2 = th**2;
  th4 = th**4;
  th6 = th**6;
  th8 = th**8;
  thd = th * (1.0 + k1 * th2 + k2 * th4 + k3 * th6 + k4 * th8);

  th_r = 1.0 / (r * r + 1.0);
  thd_th = 1.0 + 3.0 * k1 * th2 + 5.0 * k2 * th4 + 7.0 * k3 * th6 + 9.0 * k4 * th8;
  s = thd / r;
  s_r = thd_th * th_r / r - thd / (r * r);
  r_x = 1.0 / r * x;
  r_y = 1.0 / r * y;

  J_point(1, 1) = s + x * s_r * r_x;
  J_point(1, 2) = x * s_r * r_y;
  J_point(2, 1) = y * s_r * r_x;
  J_point(2, 2) = s + y * s_r * r_y;
endfunction
