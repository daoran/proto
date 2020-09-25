function factor = imu_factor_init(ts, param_ids, imu_buf, covar=eye(2))
  factor.ts = ts;
  factor.param_ids = param_ids;
  factor.imu_buf = imu_buf;
  factor.covar = covar;
endfunction
