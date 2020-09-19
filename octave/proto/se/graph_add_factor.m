function graph = graph_add_factor(graph, factor)
  graph.factors{length(graph.factors) + 1} = factor;
endfunction
