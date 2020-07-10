function pid = pid_init(k_p, k_i, k_d)
  pid = {};

  pid.k_p = k_p;
  pid.k_i = k_i;
  pid.k_d = k_d;

  pid.error_sum = 0.0;
  pid.error_prev = 0.0;
  pid.error_p = 0.0;
  pid.error_i = 0.0;
  pid.error_d = 0.0;

endfunction
