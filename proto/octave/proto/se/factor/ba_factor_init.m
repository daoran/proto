function factor = ba_factor_init(ts, param_ids, z, covar=eye(2))
  assert(length(param_ids) == 3);
  assert(size(z) == [2, 1]);
  assert(size(covar) == [2, 2]);

  factor.type = "ba_factor";
  factor.ts = ts;
  factor.param_ids = param_ids;
  factor.z = z;
  factor.covar = covar;
  factor.sqrt_info = chol(inv(covar));
  factor.eval = @ba_factor_eval;
endfunction
