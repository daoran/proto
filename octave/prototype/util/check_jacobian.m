function retval = check_jacobian(jac_name, fdiff, jac, threshold, print=false)
  delta = sqrt(sum((fdiff - jac)(:))**2);
  if (delta > threshold)
    retval = -1;
    if print
      printf("Check [%s] failed!\n", jac_name);
    endif
    fdiff_minus_jac = fdiff - jac
    fdiff
    jac
    delta
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
