function factor = ba_factor_init(ts, param_ids, z, covar=eye(2))
  factor.ts = ts;
  factor.param_ids = param_ids;
  factor.z = z;
  factor.covar = covar;
endfunction
