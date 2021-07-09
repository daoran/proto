function [retval] = perturb_pose(T, step_size, i)
  assert(size(T) == [4, 4] && i >= 0 && i <= 6);
  if i <= 3
    retval = perturb_trans(T, step_size, i);
  else
    retval = perturb_rot(T, step_size, i - 3);
  endif
endfunction
