function retval = check_de__dtheta_WS(T_WS, T_SC0, T_WF, p_F, de__dtheta_WS, step_size, threshold)
  fdiff = zeros(2, 3);
  e = residual(T_WS, T_SC0, T_WF, p_F);

  for i = 1:3
    T_WS_diff = perturb_rot(T_WS, step_size, i);
    e_prime = residual(T_WS_diff, T_SC0, T_WF, p_F);
    fdiff(1:2, i) = (e_prime - e) / step_size;
  endfor

  retval = check_jacobian("de__dtheta_WS", fdiff, de__dtheta_WS, threshold);
endfunction
