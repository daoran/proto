function T = tf(R, t)
  T = eye(4, 4);
  T(1:3, 1:3) = R;
  T(1:3, 4) = t;
endfunction
