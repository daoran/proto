function [tf_diff] = perturb_trans(tf, step_size, i)
  dr = eye(3) * step_size;
  C = tf_rot(tf);
  r = tf_trans(tf);
  r_diff = r + dr(1:3, i);
  tf_diff = transform(C, r_diff);
endfunction
