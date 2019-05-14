function [T_diff] = perturb_rot(T, step_size, i)
  rvec = eye(3) * step_size;
  C = tf_rot(T);
  r = tf_trans(T);

  C_diff = rvec2rot(rvec(1:3, i));
  C_diff = C_diff * C;

  T_diff = tf(C_diff, r);
endfunction
