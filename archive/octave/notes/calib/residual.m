% Residual function
function r = residual(T_WS, T_SC0, T_WF, p_F)
  z = [0.0; 0.0];
  zhat = h(T_WS, T_SC0, T_WF, p_F);
  r = z - zhat;
endfunction
