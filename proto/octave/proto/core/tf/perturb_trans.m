function [T_diff] = perturb_trans(T, step_size, i)
  assert(size(T) == [4, 4] || size(T) == [3, 1]);

  if (size(T) == [4, 4])
    dr = eye(3) * step_size;
    C = tf_rot(T);
    r = tf_trans(T);
    r_diff = r + dr(1:3, i);
    T_diff = tf(C, r_diff);

  elseif (size(T) == [3, 1])
    r = T;
    dr = eye(3) * step_size;
    r_diff = r + dr(1:3, i);
    T_diff = r_diff;

  endif
endfunction
