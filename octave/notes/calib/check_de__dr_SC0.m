function retval = check_de__dr_SC0(T_WS, T_SC0, T_WF, p_F, de__dr_SC0, step_size, threshold)
  fdiff = zeros(2, 3);
  e = residual(T_WS, T_SC0, T_WF, p_F);

  for i = 1:3
    T_SC0_diff = perturb_trans(T_SC0, step_size, i);
    e_prime = residual(T_WS, T_SC0_diff, T_WF, p_F);
    fdiff(1:2, i) = (e_prime - e) / step_size;
  endfor

  retval = check_jacobian("de__dr_SC0", fdiff, de__dr_SC0, threshold);
endfunction
