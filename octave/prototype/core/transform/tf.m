function T = tf(C, r)
  T = eye(4, 4);
  T(1:3, 1:3) = C;
  T(1:3, 4) = r;
endfunction
