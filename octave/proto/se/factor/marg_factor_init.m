function marg_factor = marg_factor_init(graph, marg_param_ids)
  marg_factor.type = "marg_factor";
  marg_factor.factors = [];
  marg_factor.param_ids = [];
  marg_factor.marg_param_ids = marg_param_ids;
  marg_factor.eval = @marg_factor_eval;

  for i = 1:length(graph.factors)
    factor = graph.factors(i);
    marg_factor.param_ids = [marg_factor.param_ids, factor.param_ids];
    if length(intersect(factor.param_ids, marg_param_ids))
      marg_factor.factors = [marg_factor.factors, factor];
    endif
  endfor
  marg_factor.param_ids = unique(marg_factor.param_ids);
endfunction
