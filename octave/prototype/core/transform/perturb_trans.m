function [T_diff] = perturb_trans(T, step_size, i)
  dr = eye(3) * step_size;
  C = tf_rot(T);
  r = tf_trans(T);
  r_diff = r + dr(1:3, i);
  T_diff = tf(C, r_diff);
endfunction
