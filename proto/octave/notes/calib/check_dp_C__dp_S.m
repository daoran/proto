function retval = check_dp_C__dp_S(T_WS, T_SC0, T_WF, p_F, dp_C__dp_S, step_size, threshold)
  p_C = (inv(T_WS * T_SC0) * T_WF * homogeneous(p_F))(1:3);
  p_S = (T_SC0 * homogeneous(p_C))(1:3);
  p_W = (T_WS * homogeneous(p_S))(1:3);
  T_C0S = inv(T_SC0);

  step = eye(3) * step_size;
  fdiff = zeros(3, 3);
  for i = 1:3
    p_S_diff = p_S + step(1:3, i);
    p_C_diff = (T_C0S * [p_S_diff; 1])(1:3);
    fdiff(1:3, i) = (p_C_diff - p_C) / step_size;
  endfor

  retval = check_jacobian("dp_C__dp_S", fdiff, dp_C__dp_S, threshold);
endfunction
