function marg_factor = marg_factor_init(graph, marg_param_ids)
  marg_factor.type = "marg_factor";
  marg_factor.factors = [];
  marg_factor.param_ids = [];
  marg_factor.marg_param_ids = marg_param_ids;
  marg_factor.marg_param_sizes = [];
  marg_factor.eval = @marg_factor_eval;

  % Obtain marg param sizes
  marg_param_sizes = [];
  for i = 1:length(marg_param_ids)
    param_id = marg_param_ids(i);
    marg_param_sizes = [marg_param_sizes, graph.params{param_id}.min_dims];
  endfor
  marg_factor.marg_param_sizes = marg_param_sizes;

  for i = 1:length(graph.factors)
    factor = graph.factors(i);
    if length(intersect(factor.param_ids, marg_param_ids))
      marg_factor.param_ids = [marg_factor.param_ids; factor.param_ids];
      marg_factor.factors = [marg_factor.factors; factor];
    endif
  endfor
  % marg_factor.param_ids = unique(marg_factor.param_ids);
endfunction
