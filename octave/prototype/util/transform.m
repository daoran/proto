function T = transform(R, t)
  T = zeros(4, 4);
  T(1:3, 1:3) = R;
  T(1:3, 4) = t;
  T(4, 4) = 1.0;
endfunction
