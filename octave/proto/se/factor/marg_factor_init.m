function marg_factor = marg_factor_init(factors, marg_param_indicies)
  marg_factor.type = "marg_factor";
  marg_factor.factors = factors;

  marg_factor.param_ids = [];    % Parameter ids
  marg_factor.m_param_ids = {};  % Parameter ids to be marginalized
  marg_factor.r_param_ids = {};  % Parameter ids to remain
  for i = 1:length(factors)
    param_ids = factors{i}.param_ids;
    marg_factor.m_param_ids{i} = marg_param_indicies{i};
    marg_factor.r_param_ids{i} = setdiff(param_ids, marg_param_indicies{i});
    marg_factor.param_ids = [marg_factor.param_ids, param_ids];
  endfor
  marg_factor.param_ids = unique(marg_factor.param_ids);

  marg_factor.eval = @marg_factor_eval;
endfunction
