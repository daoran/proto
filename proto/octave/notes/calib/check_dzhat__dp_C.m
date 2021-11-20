function retval = check_dzhat__dp_C(T_WS, T_SC0, T_WF, p_F, dh__dp_C, step_size, threshold)
  p_C = (inv(T_WS * T_SC0) * T_WF * homogeneous(p_F))(1:3);
  z = [p_C(1) / p_C(3); p_C(2) / p_C(3)];

  step = eye(3) * step_size;
  fdiff = zeros(2, 3);
  for i = 1:3
    p_C_diff = p_C + step(1:3, i);
    z_prime = [p_C_diff(1) / p_C_diff(3); p_C_diff(2) / p_C_diff(3)];
    fdiff(1:2, i) = (z_prime - z) / step_size;
  endfor

  retval = check_jacobian("dh__dp_C", fdiff, dh__dp_C, threshold);
endfunction
