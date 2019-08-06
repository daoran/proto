function retval = check_jacobian(jac_name, fdiff, jac, threshold, print=false)
  % delta = sqrt(sum((fdiff - jac)(:))**2);
  % if (delta > threshold)

  d = (fdiff - jac);
  failed = false;

  for i = 1:rows(d)
    for j = 1:columns(d)
      delta = d(i, j);
      if (delta > threshold)
        failed = true;
      endif
    endfor
  endfor

  if failed
    retval = -1;
    if print
      printf("Check [%s] failed!\n", jac_name);
    endif
    fdiff_minus_jac = fdiff - jac
    num_diff = fdiff
    jac
    % delta
    if print
      printf("----------------------------------------\n");
    endif
  else
    if print
      printf("Check [%s] passed!\n", jac_name);
    endif
    retval = 0;
  endif
endfunction
