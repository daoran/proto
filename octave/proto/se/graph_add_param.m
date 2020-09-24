function [graph, param_id] = graph_add_param(graph, param)
  param_id = length(graph.params) + 1;
  param.id = param_id;
  graph.params{param_id} = param;
endfunction
