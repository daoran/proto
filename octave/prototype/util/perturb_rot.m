function [tf_diff] = perturb_rot(tf, step_size, i)
  rvec = eye(3) * step_size;
  C = tf(1:3, 1:3);
  r = tf(1:3, 4);

  C_diff = rvec2rot(rvec(1:3, i));
  C_diff = C_diff * C;

  tf_diff = transform(C_diff, r);
endfunction
