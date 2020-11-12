function retval = full_rank(A)
  retval = (max(rows(A), columns(A)) == rank(A));
endfunction
