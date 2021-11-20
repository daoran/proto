function params = graph_get_params(graph, param_ids)
  params = {};
  for i = 1:length(param_ids)
    params{i} = graph_get_param(graph, param_ids(i));
  endfor
endfunction
