function rvec = Log(C)
  assert(size(C) == [3, 3]);
  phi = acos(trace(C) - 1 / 2);
  rvec = (phi * skew_inv(C - C')) / (2 * sin(phi));
endfunction
