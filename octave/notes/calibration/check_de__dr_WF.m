function retval = check_de__dr_WF(T_WS, T_SC0, T_WF, p_F, de__dr_WF, step_size, threshold)
  fdiff = zeros(2, 3);
  e = residual(T_WS, T_SC0, T_WF, p_F);

  for i = 1:3
    T_WF_diff = perturb_trans(T_WF, step_size, i);
    e_prime = residual(T_WS, T_SC0, T_WF_diff, p_F);
    fdiff(1:2, i) = (e_prime - e) / step_size;
  endfor

  retval = check_jacobian("de__dr_WF", fdiff, de__dr_WF, threshold);
endfunction
