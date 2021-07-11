function [retval] = perturb_rot(T, step_size, i)
  assert(size(T) == [4, 4] || size(T) == [3, 3]);

  rvec = eye(3) * step_size;
  dC = rvec2rot(rvec(1:3, i));

  if (size(T) == [4, 4])
    C = tf_rot(T);
    r = tf_trans(T);

    % q = tf_quat(T);
    % q_diff = [1; rvec(1, i) / 2; rvec(2, i) / 2; rvec(3, i) / 2];
    % C_diff = quat2rot(quat_mul(q, q_diff))

    C_diff = C * dC;
    retval = tf(C_diff, r);

  elseif (size(T) == [3, 3])
    C = T;
    retval = C * dC;

  endif
endfunction
