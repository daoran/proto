function [graph, factor_id] = graph_add_factor(graph, factor)
  assert(isfield(factor, "param_ids"));
  assert(length(factor.param_ids) > 0);

  for i = 1:length(factor.param_ids)
    graph_check_param(graph, factor.param_ids(i));
  endfor
  factor_id = length(graph.factors) + 1;
  factor.id = factor_id;
  graph.factors{factor_id} = factor;
endfunction
