function graph = graph_add_factor(graph, factor)
  assert(isfield(factor, "param_ids"));
  assert(length(factor.param_ids) > 0);
  graph.factors{length(graph.factors) + 1} = factor;
endfunction
