function covar = estimate_covariance(H, param_idx, param_size)
  covar = pinv(H);

  start_idx = param_idx;
  end_idx = start_idx + param_size - 1;
  covar = covar(start_idx:end_idx, start_idx:end_idx);
endfunction
