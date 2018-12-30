function Omega = quat_omega(w)
  Omega = [-skew(w), w;
           -transpose(w), 0.0];
endfunction
