function q_conj = quat_conj(q)
  qw = q(1);
  qx = q(2);
  qy = q(3);
  qz = q(4);
  q_conj = [qw; -qx; -qy; -qz];
endfunction
