function params = graph_get_params(graph, param_ids)
  params = {};
  for i = 1:length(param_ids)
    params{i} = graph.params{param_ids(i)};
  endfor
endfunction
