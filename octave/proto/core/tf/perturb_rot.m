function [retval] = perturb_rot(T, step_size, i)
  assert(size(T) == [4, 4]);
  rvec = eye(3) * step_size;

  C = tf_rot(T);
  r = tf_trans(T);

  % Note: using right-multiply
  q_diff = quat_mul(tf_quat(T), quat_delta(rvec(1:3, i)));
  q_diff = quat_normalize(q_diff);
  C_diff = quat2rot(q_diff);

  retval = tf(C_diff, r);
endfunction
