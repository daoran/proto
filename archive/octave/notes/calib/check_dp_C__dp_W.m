function retval = check_dp_C__dp_W(T_WS, T_SC0, T_WF, p_F, dp_C__dp_W, step_size, threshold)
  p_C = (inv(T_WS * T_SC0) * T_WF * homogeneous(p_F))(1:3);
  p_S = (T_SC0 * homogeneous(p_C))(1:3);
  p_W = (T_WS * homogeneous(p_S))(1:3);
  T_C0W = inv(T_WS * T_SC0);

  step = eye(3) * step_size;
  fdiff = zeros(3, 3);
  for i = 1:3
    p_W_diff = p_W + step(1:3, i);
    p_C_diff = (T_C0W * [p_W_diff; 1])(1:3);
    fdiff(1:3, i) = (p_C_diff - p_C) / step_size;
  endfor

  retval = check_jacobian("dp_C__dp_W", fdiff, dp_C__dp_W, threshold);
endfunction
