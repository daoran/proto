function factor = pose_factor_init(ts, param_ids, pose, covar=eye(6))
  assert(length(param_ids) == 1);
  assert(size(pose) == [4, 4]);
  assert(size(covar) == [6, 6]);

  factor.type = "pose_factor";
  factor.ts = ts;
  factor.param_ids = param_ids;
  factor.pose = pose;
  factor.covar = covar;
  factor.sqrt_info = chol(inv(covar));
  factor.eval = @pose_factor_eval;
endfunction
