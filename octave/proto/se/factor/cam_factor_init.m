function factor = cam_factor_init(ts, param_ids, z, covar=eye(2))
  assert(length(param_ids) == 4);
  assert(size(z) == [2, 1]);
  assert(size(covar) == [2, 2]);

  factor.ts = ts;
  factor.param_ids = param_ids;
  factor.z = z;
  factor.covar = covar;
endfunction
