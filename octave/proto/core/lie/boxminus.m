function alpha = boxplus(C_a, C_b)
  % alpha = C_a [-] C_b
  alpha = Log(inv(C_b) * C_a);
endfunction
