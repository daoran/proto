function factor = marg_factor_init(ts, param_ids, z)
  factor.type = "marg_factor";
  factor.param_ids = param_ids;
  factor.z = z;
  factor.eval = @marg_factor_eval;
endfunction
