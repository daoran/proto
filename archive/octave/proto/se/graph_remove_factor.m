function graph = graph_remove_factor(graph, factor_id)
  factors = [];

  for i = 1:length(graph.factors)
    if graph.factors(i).id != factor_id
      factors = [factors, graph.factors(i)];
    endif
  endfor

  graph.factors = factors;
endfunction
