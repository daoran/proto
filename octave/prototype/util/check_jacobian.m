function retval = check_jacobian(jac_name, fdiff, jac, threshold)
  delta = sqrt(sum((fdiff - jac)(:))**2);
  if (delta > threshold)
    retval = -1;
    printf("Check [%s] failed!\n", jac_name);
    fdiff_minus_jac = fdiff - jac
    fdiff
    jac
    delta
    printf("----------------------------------------\n");
  else
    printf("Check [%s] passed!\n", jac_name);
    retval = 0;
  endif
endfunction
