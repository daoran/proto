function q_inv = quat_inv(q)
  % assert(norm(q) == 1.0);
  q_inv = quat_conj(q);
endfunction
