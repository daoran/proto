function retval = isapprox(A, B, threshold=1e-5)
  retval = all(all(abs(A - B) < threshold));
endfunction
