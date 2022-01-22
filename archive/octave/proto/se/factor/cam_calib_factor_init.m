function factor = cam_calib_factor_init(ts, param_ids, z, r_FFi, covar=eye(2))
  assert(length(param_ids) == 3);
  assert(size(z) == [2, 1]);
  assert(size(covar) == [2, 2]);

  factor.type = "cam_calib_factor";
  factor.ts = ts;
  factor.param_ids = param_ids;
  factor.z = z;
  factor.r_FFi = r_FFi;
  factor.covar = covar;
  factor.sqrt_info = chol(inv(covar));
  factor.eval = @cam_calib_factor_eval;
endfunction
