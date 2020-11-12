function [retval] = perturb_rot(T, step_size, i)
  assert(size(T) == [4, 4] || size(T) == [3, 3]);

  if (size(T) == [4, 4])
    C = tf_rot(T);
    r = tf_trans(T);

    rvec = eye(3) * step_size;
    C_diff = rvec2rot(rvec(1:3, i));
    C_diff = C_diff * C;
    retval = tf(C_diff, r);

  elseif (size(T) == [3, 3])
    rvec = eye(3) * step_size;
    C = T;
    C_diff = rvec2rot(rvec(1:3, i));
    retval = C_diff * C;

  endif
endfunction
