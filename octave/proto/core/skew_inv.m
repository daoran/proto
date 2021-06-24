function w = skew_inv(A)
  assert(size(A) == [3, 3]);
  w = [A(3, 2), A(1, 3), A(2, 1)];
endfunction
