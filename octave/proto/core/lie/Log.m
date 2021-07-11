function rvec = Log(C)
  assert(size(C) == [3, 3]);
  phi = acos((trace(C) - 1) / 2);
  u = skew_inv(C - C') / (2 * sin(phi));
  rvec = phi * u;
endfunction
