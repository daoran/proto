function [T_perturbed] = perturb_pose(T, step_size, i)
  assert(size(T) == [4, 4] && i >= 0 && i <= 6);
  if i <= 3
    T_perturbed = perturb_trans(T, step_size, i);
  else
    T_perturbed = perturb_rot(T, step_size, i - 3);
  endif
endfunction
